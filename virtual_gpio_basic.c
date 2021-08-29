// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the QEMU ivshmem GPIO PCI device
 *
 * Copyright (c) 2021 Nikita Shubin
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
 */
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

static int nirqs = 32;
module_param(nirqs, int, 0644);
MODULE_PARM_DESC(nirqs, "number of interrupts to allocate");

/*
 * ============================================================
 *                         BASIC STUFF
 * ============================================================
 */
struct virtual_gpio {
	spinlock_t lock;
	struct pci_dev *pdev;
	/* gpiolib */
	struct gpio_chip chip;
	/* data mmio region */
	void __iomem    *data_base_addr;

	/* irq handling */
	unsigned int irq;
	struct irq_chip ic;
};

static struct pci_device_id virtual_gpio_id_table[] = {
	{ 0x1af4, 0x1110, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 32 },
	{ 0 },
};
MODULE_DEVICE_TABLE(pci, virtual_gpio_id_table);

/*  relevant control register offsets */
enum {
	IntrMask        = 0x00,    /* Interrupt Mask */
	IntrStatus      = 0x04,    /* Interrupt Status */
};

/*
 * ============================================================
 *                  UTILITY STUFF
 * ============================================================
 */

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

/*
 * ============================================================
 *                  IRQ STUFF
 * ============================================================
 */
static void virtual_gpio_interrupt(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct virtual_gpio *vg = gpiochip_get_data(gc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long i, pending;

	chained_irq_enter(irqchip, desc);
	pending = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_ST);
	/* check if irq is really raised */
	if (pending)
		for_each_set_bit(i, &pending, vg->chip.ngpio)
			generic_handle_irq(irq_find_mapping(vg->chip.irq.domain, i));

	chained_irq_exit(irqchip, desc);
}

static void virtual_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct virtual_gpio *vg = to_virtual_gpio(gc);
	unsigned long flags;
	u8 nr = d->hwirq;
	u8 mask = 1 << nr;

	spin_lock_irqsave(&vg->lock, flags);
	gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EOI, mask);
	spin_unlock_irqrestore(&vg->lock, flags);
}

static void virtual_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct virtual_gpio *vg = to_virtual_gpio(gc);
	u8 mask;
	unsigned long flags;
	u8 nr = d->hwirq;

	spin_lock_irqsave(&vg->lock, flags);
	mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN);
	mask &= ~(1 << nr);
	gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN, mask);
	spin_unlock_irqrestore(&vg->lock, flags);
}

static void virtual_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct virtual_gpio *vg = to_virtual_gpio(gc);
	u8 mask;
	unsigned long flags;
	u8 nr = d->hwirq;

	spin_lock_irqsave(&vg->lock, flags);
	mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN);
	mask |= (1 << nr);
	gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN, mask);
	spin_unlock_irqrestore(&vg->lock, flags);
}

/*
 * gpio_int_type1 controls whether the interrupt is level (0) or
 * edge (1) triggered, while gpio_int_type2 controls whether it
 * triggers on low/falling (0) or high/rising (1).
 */
static int virtual_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct virtual_gpio *vg = to_virtual_gpio(gc);
	unsigned long flags;
	int retval = 0;
	int offset = d->hwirq & (BIT(VIRTUAL_GPIO_NR_GPIOS) - 1);
	int port_mask = BIT(offset);
	u32 mask;

	dev_dbg(&vg->pdev->dev, "%s : irq=%u hwirq=%lu type=%u\n", __func__, d->irq, d->hwirq, type);

	gpio_direction_input(gc->base + d->hwirq);

	spin_lock_irqsave(&vg->lock, flags);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING, mask | port_mask);
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING, mask & ~port_mask);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING, mask | port_mask);
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING, mask & ~port_mask);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_RISING, mask | port_mask);
		mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING);
		gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_FALLING, mask | port_mask);
		break;
	default:
		retval = -EINVAL;
		goto end;
	}

	/* enable interrupt */
	mask = gc->read_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN);
	gc->write_reg(vg->data_base_addr + VIRTUAL_GPIO_INT_EN, mask | port_mask);
end:
	spin_unlock_irqrestore(&vg->lock, flags);
	return retval;
}

/* ============================================================
 *                  PROBE/REMOVE STUFF
 * ============================================================
 */
static int virtual_gpio_setup(struct pci_dev *pdev,
				  struct virtual_gpio *vg,
				  int irq)
{
	void __iomem *data = vg->data_base_addr + VIRTUAL_GPIO_DATA;
	void __iomem *dir = vg->data_base_addr + VIRTUAL_GPIO_OUT_EN;
	struct gpio_chip *gc = &vg->chip;
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *girq = &gc->irq;
	struct irq_chip *ic = &vg->ic;
	int err;

	gc->label = dev_name(&vg->pdev->dev);
	gc->owner = THIS_MODULE;
	gc->can_sleep = 0; // gpio never sleeps!
	gc->dbg_show = NULL;
#ifdef CONFIG_DEBUG_FS
	gc->dbg_show = virtual_gpio_dbg_show;
#endif

	err = bgpio_init(gc, dev, BITS_TO_BYTES(VIRTUAL_GPIO_NR_GPIOS), data, NULL, NULL, dir, NULL, 0);
	if (err)
		return err;

	ic->irq_ack = virtual_gpio_irq_ack;
	ic->irq_mask = virtual_gpio_irq_mask;
	ic->irq_unmask = virtual_gpio_irq_unmask;
	ic->irq_set_type = virtual_gpio_irq_type;
	girq->chip = ic;

	girq->parent_handler = virtual_gpio_interrupt;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, girq->num_parents,
				     sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;

	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_edge_irq;
	girq->parents[0] = irq;

	vg->irq = irq;

	return devm_gpiochip_add_data(dev, gc, vg);
}

static int virtual_gpio_probe(struct pci_dev *pdev,
			const struct pci_device_id *pdev_id)
{
	struct virtual_gpio *vg;
	struct device *dev = &pdev->dev;
	int nvecs;
	int err = 0;
	int irq;
	void __iomem *regs_base_addr;

	pr_err("%s : hello\n", __func__);

	vg = devm_kzalloc(&pdev->dev, sizeof(*vg), GFP_KERNEL);

	if (!vg)
		return -ENOMEM;

	pci_set_drvdata(pdev, vg);

	vg->pdev = pdev;
	spin_lock_init(&vg->lock);

	err = pcim_enable_device(pdev);
	if (err)
		return err;

	err = pcim_iomap_regions(pdev, BIT(2) | BIT(0), VIRTUAL_GPIO_DEV_NAME);
	if (err) {
		dev_err(dev, "I/O memory mapping error\n");
		return err;
	}

	/* BAR2: data mmio region */
	vg->data_base_addr = pcim_iomap_table(pdev)[2];

	/* BAR0: control registers */
	regs_base_addr = pcim_iomap_table(pdev)[0];

	/* interrupts: set all masks */
	iowrite32(0xffff, regs_base_addr + IntrMask);

	/* Release the IO mapping, since we already get the info from BAR0 */
	pcim_iounmap_regions(pdev, BIT(0));

	if (nirqs < 1)
		nirqs = 1;

	nvecs = pci_alloc_irq_vectors(pdev, 1, nirqs, PCI_IRQ_ALL_TYPES);
	if (nvecs < 0)
		return nvecs;

	dev_dbg(dev, "nvecs=%d\n", nvecs);

	irq = pci_irq_vector(pdev, 0);
	if (irq < 0)
		return irq;

	err = virtual_gpio_setup(pdev, vg, irq);
	if (err)
		return err;

	dev_info(dev, "loaded\n");

	return 0;
}

static struct pci_driver virtual_gpio_pci_driver = {
	.name      = VIRTUAL_GPIO_DEV_NAME,
	.id_table  = virtual_gpio_id_table,
	.probe     = virtual_gpio_probe,
};

/* ============================================================
 *                  MODULE INIT/EXIT
 * ============================================================
 */
module_pci_driver(virtual_gpio_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Shubin <maquefel@gmail.com>");
MODULE_DESCRIPTION("QEMU ivshmem virtual pci device");
