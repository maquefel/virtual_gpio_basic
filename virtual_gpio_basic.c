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


#include "virtual_gpio_basic.h"

/* ============================================================
 *                         PCI SPECIFIC 
 * ============================================================ */
#include <linux/pci.h>

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
};

static int modparam_gpiobase = -1/* dynamic */;
module_param_named(gpiobase, modparam_gpiobase, int, 0444);
MODULE_PARM_DESC(gpiobase, "The GPIO number base. -1 means dynamic, which is the default.");

static struct pci_device_id virtual_gpio_id_table[] = {
        { 0x1af4, 0x1110, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
        { 0 },
};
MODULE_DEVICE_TABLE (pci, virtual_gpio_id_table);

#define IVSHMEM_IRQ_ID "ivshmem_irq_id"

#define vgwrite(dat, adr)	iowrite8((dat), vg->data_base_addr+(adr))
#define vgread(adr)         ioread8(vg->data_base_addr+(adr))

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

        struct virtual_gpio *vg = (struct virtual_gpio *)data;

        status = ioread32(vg->regs_base_addr + IntrStatus);

        if (!status || (status == 0xFFFFFFFF))
                return IRQ_NONE;

        printk(KERN_INFO "VGPIO: interrupt (status = 0x%04x)\n", status);

        /* check if irq is really raised */
        //generic_handle_irq(irq);
        return IRQ_HANDLED;
}

/* ============================================================
 *                  GPIO STUFF
 * ============================================================ */

static int virtual_gpio_direction_input(struct gpio_chip *gpio, unsigned nr)
{
    struct virtual_gpio *vg = to_virtual_gpio(gpio);
    unsigned long flags;
    u8 outen, data;

    spin_lock_irqsave(&vg->lock, flags);

    data = vgread(VIRTUAL_GPIO_DATA);
    data &= ~(1 << nr);
    vgwrite(data, VIRTUAL_GPIO_DATA);

    outen = vgread(VIRTUAL_GPIO_OUT_EN);
    outen &= ~(1 << nr);
    vgwrite(outen, VIRTUAL_GPIO_OUT_EN);

    spin_unlock_irqrestore(&vg->lock, flags);

    return 0;
}

static int virtual_gpio_get(struct gpio_chip *gpio, unsigned nr)
{
    struct virtual_gpio *vg = to_virtual_gpio(gpio);
    unsigned long flags;
    u8 data;

    spin_lock_irqsave(&vg->lock, flags);
    data = vgread(VIRTUAL_GPIO_DATA);
    spin_unlock_irqrestore(&vg->lock, flags);

    return !!(data & (1 << nr));
}

static int virtual_gpio_direction_output(struct gpio_chip *gpio,
                    unsigned nr, int val)
{
    struct virtual_gpio *vg = to_virtual_gpio(gpio);
    unsigned long flags;
    u8 outen, data;    

    spin_lock_irqsave(&vg->lock, flags);

    outen = vgread(VIRTUAL_GPIO_OUT_EN);
    outen |= (1 << nr);
    vgwrite(outen, VIRTUAL_GPIO_OUT_EN);

    data = vgread(VIRTUAL_GPIO_DATA);
    if (val)
        data |= (1 << nr);
    else
        data &= ~(1 << nr);
    vgwrite(data, VIRTUAL_GPIO_DATA);

    spin_unlock_irqrestore(&vg->lock, flags);

    return 0;
}

static void virtual_gpio_set(struct gpio_chip *gpio,
                unsigned nr, int val)
{
    struct virtual_gpio *vg = to_virtual_gpio(gpio);
    unsigned long flags;
    u8 data;

    spin_lock_irqsave(&vg->lock, flags);

    data = vgread(VIRTUAL_GPIO_DATA);

    if (val)
        data |= (1 << nr);
    else
        data &= ~(1 << nr);

    vgwrite(data, VIRTUAL_GPIO_DATA);

    spin_unlock_irqrestore(&vg->lock, flags);
}

/* ============================================================
 *                  PROBE/REMOVE STUFF
 * ============================================================ */

static void virtual_gpio_setup(struct virtual_gpio *vg)
{
    struct gpio_chip *chip = &vg->chip;

    chip->label = dev_name(&vg->pdev->dev);
    chip->owner = THIS_MODULE;
    chip->direction_input = virtual_gpio_direction_input;
    chip->get = virtual_gpio_get;
    chip->direction_output = virtual_gpio_direction_output;
    chip->set = virtual_gpio_set;
    chip->dbg_show = NULL;
    chip->base = modparam_gpiobase;
    chip->ngpio = VIRTUAL_GPIO_NR_GPIOS;
    chip->can_sleep = 0; // gpio never sleeps!
}

static int virtual_gpio_probe(struct pci_dev *pdev,
                         const struct pci_device_id *pdev_id) 
{
        struct virtual_gpio *vg;
        int err;

        vg = kzalloc(sizeof(*vg), GFP_KERNEL);
        if (!vg)
            return -ENOMEM;

        vg->pdev = pdev;
        spin_lock_init(&vg->lock);

        struct device *dev = &pdev->dev;

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

        virtual_gpio_setup(vg);
        err = gpiochip_add(&vg->chip);

        if (err) {
            dev_err(dev, "virtual_gpio: Failed to register GPIOs\n");
            goto reg_release;
        }


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
        printk(KERN_INFO "Unregister virtual_gpio device.\n");

        struct virtual_gpio *vg = pci_get_drvdata(pdev);
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
