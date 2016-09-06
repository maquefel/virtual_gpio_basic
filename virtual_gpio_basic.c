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
    struct irq_chip_generic *gc;
    struct irq_domain *irq_domain;
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

    outen = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_OUT_EN);

    gpio = vg->chip.base;

    data = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_DATA);

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

static int virtual_gpio_to_irq(struct gpio_chip *chip, unsigned pin)
{
    struct virtual_gpio *vg = gpiochip_get_data(chip);
    return irq_create_mapping(vg->irq_domain, pin);
}

static irqreturn_t virtual_gpio_interrupt (int irq, void *data)
{
    u32 status;    
    unsigned long i, pending;

    struct virtual_gpio *vg = (struct virtual_gpio *)data;
    struct gpio_chip *gc = &vg->chip;

    status = ioread32(vg->regs_base_addr + IntrStatus);

    if (!status || (status == 0xFFFFFFFF))
        return IRQ_NONE;

    pending = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_ST);

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
    int retval = 0;

    struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = gc->private;
    struct gpio_chip *chip = &vg->chip;

    u32 mask;

    dev_dbg(&vg->pdev->dev ,"virtual_gpio_irq_type irq=%u hwirq=%lu nr=%lu\n", d->irq, d->hwirq, gc->irq_base + d->hwirq);

    spin_lock_irqsave(&vg->lock, flags);

    switch (type) {
    case IRQ_TYPE_EDGE_RISING:
        mask = chip->read_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING);
        chip->write_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING, mask | d->mask);

        mask = chip->read_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING);
        chip->write_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING, mask & ~d->mask);
        break;
    case IRQ_TYPE_EDGE_FALLING:
        mask = chip->read_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING);
        chip->write_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING, mask | d->mask);

        mask = chip->read_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING);
        chip->write_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING, mask & ~d->mask);
        break;
    default:
        retval = -EINVAL;
        goto end;
    }

    /* enable interrupt */
    mask = chip->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN);
    chip->write_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN, mask | d->mask);
end:
    spin_unlock_irqrestore(&vg->lock, flags);
    return retval;
}

/* ============================================================
 *                  PROBE/REMOVE STUFF
 * ============================================================ */

static int virtual_gpio_setup(struct virtual_gpio *vg)
{
    struct gpio_chip *chip = &vg->chip;

    chip->label = dev_name(&vg->pdev->dev);
    chip->owner = THIS_MODULE;
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
    int irq_base;

    int err;
    void __iomem *data;
    void __iomem *dir;

    vg = devm_kzalloc(&pdev->dev, sizeof(*vg), GFP_KERNEL);

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

    data = vg->data_base_addr + VIRTUAL_GPIO_DATA;
    dir = vg->data_base_addr + VIRTUAL_GPIO_OUT_EN;

    dev_dbg(dev, "bgpio_init nbytes=%u ngpio=%u\n", BITS_TO_BYTES(vg->chip.ngpio), vg->chip.ngpio);

    vg->chip.to_irq = virtual_gpio_to_irq;
    err = bgpio_init(&vg->chip, dev, BITS_TO_BYTES(vg->chip.ngpio),
                     data, NULL, NULL, dir, NULL, 0);
    if (err) {
        dev_err(dev, "Failed to register GPIOs: bgpio_init\n");
        goto reg_release;
    }

    err = devm_gpiochip_add_data(&pdev->dev, &vg->chip, vg);
    if (err) {
        dev_err(dev, "Failed to register GPIOs\n");
        goto reg_release;
    }

    vg->chip.parent = dev;

    irq_base = irq_alloc_descs(-1, 0, vg->chip.ngpio, 0);
    if (irq_base < 0) {
        err = irq_base;
        goto desc_release;
    }

    vg->irq_domain = irq_domain_add_linear(0, vg->chip.ngpio, &irq_domain_simple_ops, vg);
    if (!vg->irq_domain) {
        err = -ENXIO;
        dev_err(&pdev->dev, "cannot initialize irq domain\n");
        goto desc_release;
    }

    irq_domain_associate_many(vg->irq_domain, irq_base, 0, vg->chip.ngpio);

    gc = irq_alloc_generic_chip(VIRTUAL_GPIO_DEV_NAME, 1, irq_base, vg->data_base_addr, handle_edge_irq);
    if(!gc) {
        dev_err(dev, "irq_alloc_generic_chip failed!\n");
        goto chip_release;
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

    ct->regs.ack = VIRTUAL_GPIO_INT_EOI;
    ct->regs.mask = VIRTUAL_GPIO_INT_EN;

    irq_setup_generic_chip(gc, IRQ_MSK(VIRTUAL_GPIO_NR_GPIOS), 0, 0, 0);

    vg->chip.irqdomain = vg->irq_domain;

    gpiochip_set_chained_irqchip(&vg->chip, &ct->chip, pdev->irq, NULL);

    pci_set_drvdata(pdev, vg);

    dev_dbg(dev, "regs iomap base = 0x%lx, irq = %u\n",
             (unsigned long)vg->regs_base_addr, pdev->irq);
    dev_dbg(dev, "regs_addr_start = 0x%lx regs_len = %lu\n",
             (unsigned long)vg->regs_start,
             (unsigned long)vg->regs_len);
    dev_dbg(dev, "generic chip irq base = 0x%lx\n",
             (unsigned long)gc->irq_base);

    return 0;

chip_release:
    free_irq(pdev->irq, vg);
    irq_remove_generic_chip(vg->gc, 0, 0, 0);   
    kfree(vg->gc);
desc_release:
    irq_free_descs(gc->irq_base, vg->chip.ngpio);
    pci_iounmap(pdev, vg->regs_base_addr);
reg_release:
    pci_iounmap(pdev, vg->data_base_addr);
    pci_set_drvdata(pdev, NULL);
pci_release:
    pci_release_regions(pdev);
pci_disable:
    pci_disable_device(pdev);
    return -EBUSY;
}

static void virtual_gpio_remove(struct pci_dev* pdev)
{    
    struct virtual_gpio *vg = pci_get_drvdata(pdev);
    dev_dbg(&pdev->dev, "Unregister virtual_gpio device.\n");
    free_irq(pdev->irq, vg);
    irq_remove_generic_chip(vg->gc, IRQ_MSK(VIRTUAL_GPIO_NR_GPIOS), IRQ_NOREQUEST | IRQ_NOPROBE, 0);
    kfree(vg->gc);
    irq_free_descs(vg->gc->irq_base, vg->chip.ngpio);
    pci_iounmap(pdev, vg->regs_base_addr);
    pci_iounmap(pdev, vg->data_base_addr);
    pci_set_drvdata(pdev, NULL);
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
    pci_unregister_driver(&virtual_gpio_pci_driver);
}

module_init(virtual_gpio_init);
module_exit(virtual_gpio_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Shubin <maquefel@gmail.com>");
MODULE_DESCRIPTION("Demo module for QEMU ivshmem virtual pci device");
