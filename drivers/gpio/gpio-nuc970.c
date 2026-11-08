// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 GPIO controller
 *
 * Rewritten from drivers/gpio/gpio-nuc970.c in Nuvoton's NUC970 Linux BSP
 * (Copyright (c) 2014 Nuvoton technology corporation) for current gpiolib.
 *
 * Ten ports (A..J) of up to 16 pins, 0x40 bytes apart. One gpio_chip covers
 * all of them with a stride of 32 lines per port and a fixed base of 0, which
 * keeps the BSP's legacy numbering (gpio = port * 32 + pin, e.g. PB12 = 44,
 * PG1 = 193) for the vendor userland that still uses /sys/class/gpio.
 * Pin muxing on request goes through pinctrl (gpio-ranges).
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define NUC970_GPIO_PORTS	10
#define NUC970_GPIO_STRIDE	32	/* lines per port in the numbering */
#define NUC970_GPIO_PINS	16	/* real pins per port */
#define NUC970_GPIOJ_PINS	5

#define GPIO_PORT(p)		((p) * 0x40)
#define GPIO_DIR		0x00	/* 1 = output */
#define GPIO_DATAOUT		0x04
#define GPIO_DATAIN		0x08
#define GPIO_IMD		0x0c	/* 1 = level, 0 = edge */
#define GPIO_IREN		0x10	/* rising edge / high level */
#define GPIO_IFEN		0x14	/* falling edge / low level */
#define GPIO_ISR		0x18	/* write 1 to clear */
#define GPIO_PUEN		0x20
#define GPIO_PDEN		0x24
#define GPIO_PORT_ISR		0x3fc	/* one pending bit per port */

struct nuc970_gpio {
	struct gpio_chip gc;
	void __iomem *base;
	raw_spinlock_t lock;
	/* per-port trigger enables, applied on unmask */
	u16 rise[NUC970_GPIO_PORTS];
	u16 fall[NUC970_GPIO_PORTS];
};

static inline void __iomem *nuc970_port_reg(struct nuc970_gpio *ng,
					    unsigned int offset,
					    unsigned int reg)
{
	return ng->base + GPIO_PORT(offset / NUC970_GPIO_STRIDE) + reg;
}

static inline u32 nuc970_bit(unsigned int offset)
{
	return BIT(offset % NUC970_GPIO_STRIDE);
}

static void nuc970_rmw(struct nuc970_gpio *ng, unsigned int offset,
		       unsigned int reg, bool set)
{
	void __iomem *addr = nuc970_port_reg(ng, offset, reg);
	u32 val = readl_relaxed(addr);

	if (set)
		val |= nuc970_bit(offset);
	else
		val &= ~nuc970_bit(offset);
	writel_relaxed(val, addr);
}

static int nuc970_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);

	if (readl_relaxed(nuc970_port_reg(ng, offset, GPIO_DIR)) &
	    nuc970_bit(offset))
		return GPIO_LINE_DIRECTION_OUT;
	return GPIO_LINE_DIRECTION_IN;
}

static int nuc970_gpio_direction_input(struct gpio_chip *gc,
				       unsigned int offset)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, offset, GPIO_DIR, false);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
	return 0;
}

static int nuc970_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&ng->lock, flags);
	/* latch the level first so the pin does not glitch */
	nuc970_rmw(ng, offset, GPIO_DATAOUT, value);
	nuc970_rmw(ng, offset, GPIO_DIR, true);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
	return 0;
}

static int nuc970_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	unsigned int reg = GPIO_DATAIN;

	/* like the BSP, report the driven level for outputs */
	if (nuc970_gpio_get_direction(gc, offset) == GPIO_LINE_DIRECTION_OUT)
		reg = GPIO_DATAOUT;

	return !!(readl_relaxed(nuc970_port_reg(ng, offset, reg)) &
		  nuc970_bit(offset));
}

static int nuc970_gpio_set(struct gpio_chip *gc, unsigned int offset,
			   int value)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, offset, GPIO_DATAOUT, value);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
	return 0;
}

static int nuc970_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				  unsigned long config)
{
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	bool pu = false, pd = false;
	unsigned long flags;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_DISABLE:
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		pu = true;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		pd = true;
		break;
	default:
		return -ENOTSUPP;
	}

	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, offset, GPIO_PUEN, pu);
	nuc970_rmw(ng, offset, GPIO_PDEN, pd);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
	return 0;
}

static int nuc970_gpio_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	unsigned int port;

	bitmap_zero(valid_mask, ngpios);
	for (port = 0; port < NUC970_GPIO_PORTS; port++)
		bitmap_set(valid_mask, port * NUC970_GPIO_STRIDE,
			   port == NUC970_GPIO_PORTS - 1 ?
			   NUC970_GPIOJ_PINS : NUC970_GPIO_PINS);
	return 0;
}

/* interrupts */

static void nuc970_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);

	writel_relaxed(nuc970_bit(hw), nuc970_port_reg(ng, hw, GPIO_ISR));
}

static void nuc970_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, hw, GPIO_IREN, false);
	nuc970_rmw(ng, hw, GPIO_IFEN, false);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
	gpiochip_disable_irq(gc, hw);
}

static void nuc970_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned int port = hw / NUC970_GPIO_STRIDE;
	u32 bit = nuc970_bit(hw);
	unsigned long flags;

	gpiochip_enable_irq(gc, hw);
	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, hw, GPIO_IREN, ng->rise[port] & bit);
	nuc970_rmw(ng, hw, GPIO_IFEN, ng->fall[port] & bit);
	raw_spin_unlock_irqrestore(&ng->lock, flags);
}

static int nuc970_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned int port = hw / NUC970_GPIO_STRIDE;
	u16 bit = nuc970_bit(hw);
	bool level = type & IRQ_TYPE_LEVEL_MASK;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_LEVEL_HIGH:
		ng->rise[port] |= bit;
		ng->fall[port] &= ~bit;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_LOW:
		ng->rise[port] &= ~bit;
		ng->fall[port] |= bit;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		ng->rise[port] |= bit;
		ng->fall[port] |= bit;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&ng->lock, flags);
	nuc970_rmw(ng, hw, GPIO_IMD, level);
	raw_spin_unlock_irqrestore(&ng->lock, flags);

	irq_set_handler_locked(d, level ? handle_level_irq : handle_edge_irq);
	return 0;
}

static void nuc970_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct nuc970_gpio *ng = gpiochip_get_data(gc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long ports, pending;
	unsigned int port, pin;
	u32 isr, stray;

	chained_irq_enter(chip, desc);

	ports = readl_relaxed(ng->base + GPIO_PORT_ISR) &
		GENMASK(NUC970_GPIO_PORTS - 1, 0);
	for_each_set_bit(port, &ports, NUC970_GPIO_PORTS) {
		void __iomem *base = ng->base + GPIO_PORT(port);

		isr = readl_relaxed(base + GPIO_ISR) &
		      GENMASK(NUC970_GPIO_PINS - 1, 0);
		pending = isr & (readl_relaxed(base + GPIO_IREN) |
				 readl_relaxed(base + GPIO_IFEN));

		/*
		 * A status bit latched just before its pin was masked (e.g. a
		 * button bounce racing free_irq on "echo none > edge") would
		 * otherwise never be cleared and keep the level-triggered
		 * parent (AIC 57) asserted forever. The BSP demux cleared
		 * every ISR bit it saw; do the same for the unhandled ones.
		 */
		stray = isr & ~pending;
		if (stray)
			writel_relaxed(stray, base + GPIO_ISR);

		for_each_set_bit(pin, &pending, NUC970_GPIO_PINS)
			generic_handle_domain_irq(gc->irq.domain,
					port * NUC970_GPIO_STRIDE + pin);
	}

	chained_irq_exit(chip, desc);
}

static const struct irq_chip nuc970_gpio_irq_chip = {
	.name		= "nuc970-gpio",
	.irq_ack	= nuc970_gpio_irq_ack,
	.irq_mask	= nuc970_gpio_irq_mask,
	.irq_unmask	= nuc970_gpio_irq_unmask,
	.irq_set_type	= nuc970_gpio_irq_set_type,
	.flags		= IRQCHIP_IMMUTABLE | IRQCHIP_SKIP_SET_WAKE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int nuc970_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *girq;
	struct nuc970_gpio *ng;
	struct clk *clk;
	unsigned int port;
	int irq;

	ng = devm_kzalloc(dev, sizeof(*ng), GFP_KERNEL);
	if (!ng)
		return -ENOMEM;
	raw_spin_lock_init(&ng->lock);

	ng->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ng->base))
		return PTR_ERR(ng->base);

	/* APB clock and debounce engine clock share the PCLKEN0[3] gate */
	clk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no pclk\n");
	clk = devm_clk_get_optional_enabled(dev, "eclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "bad eclk\n");

	/* start with all pin interrupts disabled and cleared */
	for (port = 0; port < NUC970_GPIO_PORTS; port++) {
		void __iomem *base = ng->base + GPIO_PORT(port);

		writel_relaxed(0, base + GPIO_IREN);
		writel_relaxed(0, base + GPIO_IFEN);
		writel_relaxed(0xffff, base + GPIO_ISR);
	}

	ng->gc.label = "nuc970-gpio";
	ng->gc.parent = dev;
	ng->gc.owner = THIS_MODULE;
	ng->gc.base = 0;	/* legacy numbering, see above */
	ng->gc.ngpio = NUC970_GPIO_PORTS * NUC970_GPIO_STRIDE;
	ng->gc.request = gpiochip_generic_request;
	ng->gc.free = gpiochip_generic_free;
	ng->gc.get_direction = nuc970_gpio_get_direction;
	ng->gc.direction_input = nuc970_gpio_direction_input;
	ng->gc.direction_output = nuc970_gpio_direction_output;
	ng->gc.get = nuc970_gpio_get;
	ng->gc.set = nuc970_gpio_set;
	ng->gc.set_config = nuc970_gpio_set_config;
	ng->gc.init_valid_mask = nuc970_gpio_init_valid_mask;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		girq = &ng->gc.irq;
		gpio_irq_chip_set_chip(girq, &nuc970_gpio_irq_chip);
		girq->parent_handler = nuc970_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_bad_irq;
	}

	return devm_gpiochip_add_data(dev, &ng->gc, ng);
}

static const struct of_device_id nuc970_gpio_of_match[] = {
	{ .compatible = "nuvoton,nuc970-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_gpio_of_match);

static struct platform_driver nuc970_gpio_driver = {
	.probe	= nuc970_gpio_probe,
	.driver	= {
		.name		= "nuc970-gpio",
		.of_match_table	= nuc970_gpio_of_match,
	},
};

module_platform_driver(nuc970_gpio_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 GPIO driver");
MODULE_LICENSE("GPL");
