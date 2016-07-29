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
#include "virtual_gpio_basic.h"

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
};

static int modparam_gpiobase = -1/* dynamic */;
module_param_named(gpiobase, modparam_gpiobase, int, 0444);
MODULE_PARM_DESC(gpiobase, "The GPIO number base. -1 means dynamic, which is the default.");

#define PCI_SUBVENDOR_ID_MAQUEFEL 0x0010
#define PCI_SUBDEVICE_ID_VG8      0x0108
#define PCI_SUBDEVICE_ID_VG16     0x0208
#define PCI_SUBDEVICE_ID_VG32     0x0308

static struct pci_device_id virtual_gpio_id_table[] = {
//{ 0x1af4, 0x1110, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 8 },
{ 0x1af4, 0x1110, PCI_SUBVENDOR_ID_MAQUEFEL, PCI_SUBDEVICE_ID_VG8, 0, 0, 8 },
{ 0x1af4, 0x1110, PCI_SUBVENDOR_ID_MAQUEFEL, PCI_SUBDEVICE_ID_VG16, 0, 0, 16 },
{ 0x1af4, 0x1110, PCI_SUBVENDOR_ID_MAQUEFEL, PCI_SUBDEVICE_ID_VG32, 0, 0, 32 },
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

static inline int printbinary(char *buf, unsigned long x, int nbits)
{
    unsigned long mask = 1UL << (nbits - 1);
    while (mask != 0) {
        *buf++ = (mask & x ? '1' : '0');
        mask >>= 1;
    }
    *buf = '\0';

    return nbits;
}

#ifdef CONFIG_DEBUG_FS
static void virtual_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
    struct virtual_gpio *vg = to_virtual_gpio(chip);
    u8 data_reg, data_dir_reg;
    u8 outen, data;
    int gpio, is_out, i;

    outen = vgread(VIRTUAL_GPIO_OUT_EN);
    gpio = vg->chip.base;

    data = vgread(VIRTUAL_GPIO_DATA);

    is_out = 1;
    for (i = 0; i < VIRTUAL_GPIO_NR_GPIOS; i++, gpio++) {
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

static void virtual_gpio_irq_ack(struct irq_data *d)
{    
    unsigned long flags;
    u32 nr = d->hwirq;
    u32 mask = 1 << nr;

    struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = to_virtual_gpio(gc);

    spin_lock_irqsave(&vg->lock, flags);
    gc->write_reg(vg->data_base_addr + vg->reg_off.irqeoi, mask);
    spin_unlock_irqrestore(&vg->lock, flags);
}

static void virtual_gpio_irq_mask(struct irq_data *d)
{
    u32 mask;
    unsigned long flags;
    u32 nr = d->hwirq;

    struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = to_virtual_gpio(gc);

    spin_lock_irqsave(&vg->lock, flags);
    mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqen);
    mask &= ~(1 << nr);
    gc->write_reg(vg->data_base_addr + vg->reg_off.irqen, mask);
    spin_unlock_irqrestore(&vg->lock, flags);
}

static void virtual_gpio_irq_unmask(struct irq_data *d)
{
    u32 mask;
    unsigned long flags;
    u32 nr = d->hwirq;

    struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = to_virtual_gpio(gc);

    spin_lock_irqsave(&vg->lock, flags);
    mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqen);
    mask |= (1 << nr);
    gc->write_reg(vg->data_base_addr + vg->reg_off.irqen, mask);
    spin_unlock_irqrestore(&vg->lock, flags);
}

/*
 * gpio_int_type1 controls whether the interrupt is level (0) or
 * edge (1) triggered, while gpio_int_type2 controls whether it
 * triggers on low/falling (0) or high/rising (1).
 */
static int virtual_gpio_irq_type(struct irq_data *d, unsigned int type)
{
    unsigned long flags;

    struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
    struct virtual_gpio *vg = to_virtual_gpio(gc);

    irq_flow_handler_t handler;

    u32 mask;
    u32 nr = d->hwirq;
    int retval = 0;

    printk(KERN_DEBUG "virtual_gpio_irq_type irq=%u hwirq=%lu nr=%lu\n", d->irq, d->hwirq, gc->base + d->hwirq);
    /* set pin as input */
    gpio_direction_input(gc->base + d->hwirq);

    spin_lock_irqsave(&vg->lock, flags);
    switch (type) {
    case IRQ_TYPE_EDGE_RISING:
        mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqris);
        mask |= (1 << nr);
        gc->write_reg(vg->data_base_addr + vg->reg_off.irqris, mask);

        mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqfal);
        mask &= ~(1 << nr);
        gc->write_reg(vg->data_base_addr + vg->reg_off.irqfal, mask);

        handler = handle_edge_irq;
        break;
    case IRQ_TYPE_EDGE_FALLING:
        mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqfal);
        mask |= (1 << nr);
        gc->write_reg(vg->data_base_addr + vg->reg_off.irqfal, mask);

        mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqris);
        mask &= ~(1 << nr);
        gc->write_reg(vg->data_base_addr + vg->reg_off.irqris, mask);

        handler = handle_edge_irq;
        break;
    default:
        retval = -EINVAL;
        goto end;
    }

    /* enable interrupt */
    mask = gc->read_reg(vg->data_base_addr + vg->reg_off.irqen);
    mask |= (1 << nr);
    gc->write_reg(vg->data_base_addr + vg->reg_off.irqen, mask);

end:
    spin_unlock_irqrestore(&vg->lock, flags);
    return retval;
}

static struct irq_chip virtual_gpio_irq_chip = {
    .name           = "GPIO",
    .irq_ack        = virtual_gpio_irq_ack,
    .irq_mask       = virtual_gpio_irq_mask,
    .irq_unmask     = virtual_gpio_irq_unmask,
    .irq_set_type   = virtual_gpio_irq_type,
};

/* ============================================================
 *                  PROBE/REMOVE STUFF
 * ============================================================ */

static int virtual_gpio_setup(struct virtual_gpio *vg)
{
    struct gpio_chip *chip = &vg->chip;
    u8 nbytes = BITS_TO_BYTES(chip->ngpio);

    chip->label = dev_name(&vg->pdev->dev);
    chip->owner = THIS_MODULE;

    /* setup regs depending on ngpio */
    vg->reg_off.dat = nbytes*VIRTUAL_GPIO_DATA;
    vg->reg_off.dir = nbytes*VIRTUAL_GPIO_OUT_EN;
    vg->reg_off.irqen = nbytes*VIRTUAL_GPIO_INT_EN;
    vg->reg_off.irqst = nbytes*VIRTUAL_GPIO_INT_ST;
    vg->reg_off.irqeoi = nbytes*VIRTUAL_GPIO_INT_EOI;
    vg->reg_off.irqris = nbytes*VIRTUAL_GPIO_RISING;
    vg->reg_off.irqfal = nbytes*VIRTUAL_GPIO_FALLING;

    chip->base = modparam_gpiobase;    
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
    int err;
    void __iomem *data;
    void __iomem *dir;

    vg = kzalloc(sizeof(*vg), GFP_KERNEL);
    if (!vg)
        return -ENOMEM;

    vg->pdev = pdev;
    spin_lock_init(&vg->lock);

    dev = &pdev->dev;

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

    dev_info(dev, "data_mmio iomap base = 0x%lx \n",
             (unsigned long) vg->data_base_addr);

    dev_info(dev, "data_mmio_start = 0x%lx data_mmio_len = %lu\n",
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

    vg->chip.ngpio = pdev_id->driver_data;
    virtual_gpio_setup(vg);    

    data = vg->data_base_addr + vg->reg_off.dat;
    dir = vg->data_base_addr + vg->reg_off.dir;


    printk(KERN_DEBUG "bgpio_init nbytes=%u ngpio=%u\n", BITS_TO_BYTES(vg->chip.ngpio), vg->chip.ngpio);
    err = bgpio_init(&vg->chip, dev, BITS_TO_BYTES(vg->chip.ngpio),
                     data, NULL, NULL, dir, NULL, 0);

    if (err) {
        dev_err(dev, "Failed to register GPIOs: bgpio_init\n");
        goto reg_release;
    }


    err = gpiochip_add_data(&vg->chip, 0);

    if (err) {
        dev_err(dev, "Failed to register GPIOs\n");
        goto reg_release;
    }

    vg->chip.parent = dev;

    err =  gpiochip_irqchip_add(&vg->chip,
                                &virtual_gpio_irq_chip,
                                0,
                                handle_edge_irq,
                                IRQ_TYPE_NONE);

    if (err) {
        dev_err(dev, "could not connect irqchip to gpiochip\n");
        return err;
    }

    gpiochip_set_chained_irqchip(&vg->chip,
                                 &virtual_gpio_irq_chip,
                                 pdev->irq,
                                 NULL);


    dev_info(dev, "regs iomap base = 0x%lx, irq = %u\n",
             (unsigned long)vg->regs_base_addr, pdev->irq);
    dev_info(dev, "regs_addr_start = 0x%lx regs_len = %lu\n",
             (unsigned long)vg->regs_start,
             (unsigned long)vg->regs_len);

    return 0;
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
    printk(KERN_INFO "Unregister virtual_gpio device.\n");
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
