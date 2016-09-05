/*
 * file : virtual_gpio_basic.c
 * desc : demo linux device driver for the QEMU ivshmem GPIO PCI device
 *
 * Nikita Shubin
 *
 * derived from:
 * file : ne_ivshmem_basic_ldd.c
 * desc : demo linux device driver for the QEMU ivshmem PCI device
 *
 * notes: This is a skeleton version of "kvm_ivshmem.c" by
 *        Cam Macdonell <cam@cs.ualberta.ca>, Copyright 2009, GPLv2
 *        See git://gitorious.org/nahanni/guest-code.git
 *
 * Siro Mugabi, nairobi-embedded.org
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d:: " fmt, __func__, __LINE__
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/ratelimit.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/gpio/driver.h>

#include <linux/pci.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>
#endif

#define VIRTUAL_GPIO_DEV_NAME "ivshmem_gpio"
#define VIRTUAL_GPIO_NR_GPIOS 32

#define VIRTUAL_GPIO_DATA       0x00
#define VIRTUAL_GPIO_OUT_EN     0x04
#define VIRTUAL_GPIO_INT_EN     0x08
#define VIRTUAL_GPIO_INT_ST     0x0c
#define VIRTUAL_GPIO_INT_EOI    0x10
#define VIRTUAL_GPIO_RISING     0x14
#define VIRTUAL_GPIO_FALLING    0x18

#ifndef BITS_TO_BYTES
#define BITS_TO_BYTES(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE)
#endif

/* ============================================================
 *                         BASIC STUFF
 * ============================================================ */
struct virtual_gpio_offsets {
    /* basic offsets */
    u8 dat;
    u8 dir;
    /* irq offsets*/
    u8 irqen;
    u8 irqst;
    u8 irqeoi;
    u8 irqris;
    u8 irqfal;
};

struct virtual_gpio {
    spinlock_t lock;
    struct pci_dev *pdev;
    /* gpiolib */
    struct gpio_chip chip;
    /* (mmio) control registers i.e. the "register memory region" */
    void __iomem    *regs_base_addr;
    resource_size_t regs_start;
    resource_size_t regs_len;
    /* data mmio region */
    void __iomem    *data_base_addr;
    resource_size_t data_mmio_start;
    resource_size_t data_mmio_len;
    /* irq handling */
    unsigned int irq;
    struct virtual_gpio_offsets reg_off;
    struct irq_chip_generic *gc;
};

static struct pci_device_id virtual_gpio_id_table[] = {
{ 0x1af4, 0x1110, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 32 },
{ 0 },
};
MODULE_DEVICE_TABLE (pci, virtual_gpio_id_table);

/*  relevant control register offsets */
enum {
    IntrMask        = 0x00,    /* Interrupt Mask */
    IntrStatus      = 0x04,    /* Interrupt Status */
};

/* ============================================================
 *                  UTILITY STUFF
 * ============================================================ */

static inline struct virtual_gpio *to_virtual_gpio(struct gpio_chip *chip)
{
    return container_of(chip, struct virtual_gpio, chip);
}

#ifdef CONFIG_DEBUG_FS
static void virtual_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
    struct virtual_gpio *vg = to_virtual_gpio(chip);
    struct gpio_chip *gc = &vg->chip;

    u32 outen, data;
    int gpio, is_out, i;

    outen = gc->read_reg(vg->data_base_addr + vg->reg_off.dir);

    gpio = vg->chip.base;

    data = gc->read_reg(vg->data_base_addr + vg->reg_off.dat);

    is_out = 1;
    for (i = 0; i < vg->chip.ngpio; i++, gpio++) {
        seq_printf(s, " %s%d gpio-%-3d (%-12s) %s %s\n",
                   vg->chip.label, i, gpio,
                   gpiochip_is_requested(chip, i) ? : "",
                   is_out ? "out" : "in ",
                   (data & (1 << i)) ? "hi" : "lo");
    }
}
#else
#define virtual_gpio_dbg_show NULL
#endif


/* ============================================================
 *                  IRQ STUFF
 * ============================================================ */

static irqreturn_t virtual_gpio_interrupt (int irq, void *data)
{
    u32 status;    
    unsigned long i, pending;

    struct virtual_gpio *vg = (struct virtual_gpio *)data;
    struct gpio_chip *gc = &vg->chip;

    status = ioread32(vg->regs_base_addr + IntrStatus);

    if (!status || (status == 0xFFFFFFFF))
        return IRQ_NONE;

    pending = gc->read_reg(vg->data_base_addr + vg->reg_off.irqst);

    /* check if irq is really raised */
    if (pending)
    {
        for_each_set_bit(i, &pending, vg->chip.ngpio)
            generic_handle_irq(irq_find_mapping(vg->chip.irqdomain, i));
    }

    return IRQ_HANDLED;
}

/*
 * gpio_int_type1 controls whether the interrupt is level (0) or
 * edge (1) triggered, while gpio_int_type2 controls whether it
 * triggers on low/falling (0) or high/rising (1).
 */
static int virtual_gpio_irq_type(struct irq_data *d, unsigned int type)
{
    unsigned long flags;

    struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = gc->private;
    struct gpio_chip *chip = &vg->chip;

    u32 mask;

    dev_dbg(&vg->pdev->dev ,"virtual_gpio_irq_type irq=%u hwirq=%lu nr=%lu\n", d->irq, d->hwirq, gc->irq_base + d->hwirq);

    spin_lock_irqsave(&vg->lock, flags);

    switch (type) {
    case IRQ_TYPE_EDGE_RISING:
        mask = chip->read_reg(vg->data_base_addr + vg->reg_off.irqris);
        chip->write_reg(vg->data_base_addr + vg->reg_off.irqris, mask | d->mask);

        mask = chip->read_reg(vg->data_base_addr + vg->reg_off.irqfal);
        chip->write_reg(vg->data_base_addr + vg->reg_off.irqfal, mask & ~d->mask);
        break;
    case IRQ_TYPE_EDGE_FALLING:
        mask = chip->read_reg(vg->data_base_addr + vg->reg_off.irqfal);
        chip->write_reg(vg->data_base_addr + vg->reg_off.irqfal, mask | d->mask);

        mask = chip->read_reg(vg->data_base_addr + vg->reg_off.irqris);
        chip->write_reg(vg->data_base_addr + vg->reg_off.irqris, mask & ~d->mask);
        break;
    default:
        retval = -EINVAL;
        goto end;
    }

    /* enable interrupt */
    mask = chip->read_reg(vg->data_base_addr + vg->reg_off.irqen);
    chip->write_reg(vg->data_base_addr + vg->reg_off.irqen, mask | d->mask);
end:
    spin_unlock_irqrestore(&vg->lock, flags);
    return 0;
}

static int virtual_gpio_irq_reqres(struct irq_data *d)
{
    struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = gc->private;
    struct gpio_chip *chip = &vg->chip;

    if (gpiochip_lock_as_irq(chip, d->hwirq))
        return -EINVAL;

    return 0;
}

static void virtual_gpio_irq_relres(struct irq_data *d)
{
    struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = gc->private;
    struct gpio_chip *chip = &vg->chip;

    gpiochip_unlock_as_irq(chip, d->hwirq);
}

/* ============================================================
 *                  PROBE/REMOVE STUFF
 * ============================================================ */

static int virtual_gpio_setup(struct virtual_gpio *vg)
{
    struct gpio_chip *chip = &vg->chip;

    chip->label = dev_name(&vg->pdev->dev);
    chip->owner = THIS_MODULE;

    vg->reg_off.dat = VIRTUAL_GPIO_DATA;
    vg->reg_off.dir = VIRTUAL_GPIO_OUT_EN;
    vg->reg_off.irqen = VIRTUAL_GPIO_INT_EN;
    vg->reg_off.irqst = VIRTUAL_GPIO_INT_ST;
    vg->reg_off.irqeoi = VIRTUAL_GPIO_INT_EOI;
    vg->reg_off.irqris = VIRTUAL_GPIO_RISING;
    vg->reg_off.irqfal = VIRTUAL_GPIO_FALLING;

    chip->can_sleep = 0; // gpio never sleeps!
    chip->dbg_show = NULL;
#ifdef CONFIG_DEBUG_FS
    chip->dbg_show = virtual_gpio_dbg_show;
#endif
    return 0;
}

static int virtual_gpio_probe(struct pci_dev *pdev,
                              const struct pci_device_id *pdev_id)
{
    struct virtual_gpio *vg;
    struct device *dev;
    struct irq_chip_generic *gc;
    struct irq_chip_type *ct;
    struct irq_data *d;

    int err;
    void __iomem *data;
    void __iomem *dir;

    u32 msk, i;

    vg = kzalloc(sizeof(*vg), GFP_KERNEL);
    if (!vg)
        return -ENOMEM;

    vg->pdev = pdev;
    spin_lock_init(&vg->lock);

    dev = &pdev->dev;
    gc = vg->gc;

    if((err = pci_enable_device(pdev))){
        dev_err(dev, "pci_enable_device probe error %d for device %s\n",
                err, pci_name(pdev));
        return err;
    }

    if((err = pci_request_regions(pdev, VIRTUAL_GPIO_DEV_NAME)) < 0){
        dev_err(dev, "pci_request_regions error %d\n", err);
        goto pci_disable;
    }

    /* bar2: data mmio region */
    vg->data_mmio_start = pci_resource_start(pdev, 2);
    vg->data_mmio_len   = pci_resource_len(pdev, 2);
    vg->data_base_addr = ioremap_nocache(vg->data_mmio_start, vg->data_mmio_len);


    if (!vg->data_base_addr) {
        dev_err(dev, "cannot iomap region of size %lu\n",
                (unsigned long)vg->data_mmio_len);
        goto pci_release;
    }

    dev_dbg(dev, "data_mmio iomap base = 0x%lx \n",
             (unsigned long) vg->data_base_addr);

    dev_dbg(dev, "data_mmio_start = 0x%lx data_mmio_len = %lu\n",
             (unsigned long)vg->data_mmio_start,
             (unsigned long)vg->data_mmio_len);

    /* bar0: control registers */
    vg->regs_start =  pci_resource_start(pdev, 0);
    vg->regs_len = pci_resource_len(pdev, 0);
    vg->regs_base_addr = pci_iomap(pdev, 0, 0x100);

    if (!vg->regs_base_addr) {
        dev_err(dev, "cannot ioremap registers of size %lu\n",
                (unsigned long)vg->regs_len);
        goto reg_release;
    }

    pci_set_drvdata(pdev, vg);

    /* interrupts: set all masks */
    iowrite32(0xffff, vg->regs_base_addr + IntrMask);

    if (request_irq(pdev->irq,
                    virtual_gpio_interrupt,
                    IRQF_SHARED,
                    VIRTUAL_GPIO_DEV_NAME,
                    vg))
        dev_err(dev, "request_irq %d error\n", pdev->irq);

    vg->chip.ngpio = VIRTUAL_GPIO_NR_GPIOS;
    virtual_gpio_setup(vg);    

    data = vg->data_base_addr + vg->reg_off.dat;
    dir = vg->data_base_addr + vg->reg_off.dir;

    dev_dbg(dev, "bgpio_init nbytes=%u ngpio=%u\n", BITS_TO_BYTES(vg->chip.ngpio), vg->chip.ngpio);

    err = bgpio_init(&vg->chip, dev, BITS_TO_BYTES(vg->chip.ngpio),
                     data, NULL, NULL, dir, NULL, 0);

    if (err) {
        dev_err(dev, "Failed to register GPIOs: bgpio_init\n");
        goto reg_release;
    }

    err = gpiochip_add_data(&vg->chip, vg);

    if (err) {
        dev_err(dev, "Failed to register GPIOs\n");
        goto reg_release;
    }

    vg->chip.parent = dev;

    gc = irq_alloc_generic_chip(VIRTUAL_GPIO_DEV_NAME, 1, 0, vg->data_base_addr, handle_edge_irq);

    if(!gc) {
        dev_err(dev, "irq_alloc_generic_chip failed!\n");
        goto reg_release;
    }

    gc->private = vg;

    ct = gc->chip_types;
    vg->gc = gc;

    ct->type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;

    ct->chip.name = VIRTUAL_GPIO_DEV_NAME;
    ct->chip.irq_ack = irq_gc_ack_set_bit;
    ct->chip.irq_mask = irq_gc_mask_clr_bit;
    ct->chip.irq_unmask = irq_gc_mask_set_bit;
    ct->chip.irq_set_type = virtual_gpio_irq_type;

    ct->chip.irq_request_resources = virtual_gpio_irq_reqres;
    ct->chip.irq_release_resources = virtual_gpio_irq_relres;

    ct->regs.ack = vg->reg_off.irqeoi;
    ct->regs.mask = vg->reg_off.irqen;

    err =  gpiochip_irqchip_add(&vg->chip,
                                &ct->chip,
                                0,
                                handle_edge_irq,
                                IRQ_TYPE_NONE);

    if (err) {
        dev_err(dev, "Could not connect irqchip to gpiochip\n");
        goto chip_release;
    }

    gpiochip_set_chained_irqchip(&vg->chip,
                                 &ct->chip,
                                 pdev->irq,
                                 NULL);

    gc->irq_cnt = VIRTUAL_GPIO_NR_GPIOS;

    irq_setup_generic_chip(gc, 0, 0, IRQ_NOPROBE, 0);

    /* explicitly setting irq mask and chip_data */
    gc->irq_base = vg->chip.irq_base;

    msk = IRQ_MSK(VIRTUAL_GPIO_NR_GPIOS);

    for (i = gc->irq_base; msk; msk >>= 1, i++) {
        d = irq_get_irq_data(i);
        d->mask = 1 << (i - gc->irq_base);
        irq_set_chip_data(i, gc);
    }

    dev_dbg(dev, "regs iomap base = 0x%lx, irq = %u\n",
             (unsigned long)vg->regs_base_addr, pdev->irq);
    dev_dbg(dev, "regs_addr_start = 0x%lx regs_len = %lu\n",
             (unsigned long)vg->regs_start,
             (unsigned long)vg->regs_len);

    dev_dbg(dev, "generic chip irq base = 0x%lx\n",
             (unsigned long)gc->irq_base);

    return 0;

chip_release:
    irq_remove_generic_chip(vg->gc, 0, 0, 0);
    kfree(vg->gc);
reg_release:
    pci_iounmap(pdev, vg->data_base_addr);
    pci_set_drvdata(pdev, NULL);
pci_release:
    pci_release_regions(pdev);
pci_disable:
    pci_disable_device(pdev);
    kfree(vg);

    return -EBUSY;
}

static void virtual_gpio_remove(struct pci_dev* pdev)
{
    struct virtual_gpio *vg = pci_get_drvdata(pdev);
    dev_dbg(&pdev->dev, "Unregister virtual_gpio device.\n");
    irq_remove_generic_chip(vg->gc, 0, 0, 0);
    kfree(vg->gc);
    gpiochip_remove(&vg->chip);
    free_irq(pdev->irq, vg);
    pci_iounmap(pdev, vg->regs_base_addr);
    pci_iounmap(pdev, vg->data_base_addr);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
}

static struct pci_driver virtual_gpio_pci_driver = {
    .name      = VIRTUAL_GPIO_DEV_NAME,
    .id_table  = virtual_gpio_id_table,
    .probe     = virtual_gpio_probe,
    .remove    = virtual_gpio_remove,
};

/* ============================================================
 *                  MODULE INIT/EXIT
 * ============================================================ */
static int __init virtual_gpio_init (void)
{
    return pci_register_driver(&virtual_gpio_pci_driver);
}

static void __exit virtual_gpio_fini(void)
{
    pci_unregister_driver (&virtual_gpio_pci_driver);
}

module_init(virtual_gpio_init);
module_exit(virtual_gpio_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Shubin <maquefel@gmail.com>");
MODULE_DESCRIPTION("Demo module for QEMU ivshmem virtual pci device");
