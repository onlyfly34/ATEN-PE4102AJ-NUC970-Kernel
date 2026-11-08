// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 Advanced Interrupt Controller (AIC)
 *
 * Forward-ported from arch/arm/mach-nuc970/irq.c and entry-macro.S in
 * Nuvoton's NUC970 Linux BSP (Copyright (c) 2014 Nuvoton technology
 * corporation) as a DT irqchip for multiplatform kernels.
 *
 * 64 sources, all level triggered as configured by the reset defaults of
 * the source control registers. Reading IPER latches the highest priority
 * pending source into ISNR and enters it into service; writing EOSCR ends
 * the service.
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/exception.h>

#define AIC_IPER	0x118	/* interrupt priority encoding */
#define AIC_ISNR	0x120	/* interrupt source number */
#define AIC_IMR		0x128	/* interrupt mask (low) */
#define AIC_MECR	0x130	/* mask enable command (low) */
#define AIC_MECRH	0x134	/* mask enable command (high) */
#define AIC_MDCR	0x138	/* mask disable command (low) */
#define AIC_MDCRH	0x13c	/* mask disable command (high) */
#define AIC_EOSCR	0x150	/* end of service command */

#define AIC_NUM_IRQS	64

static void __iomem *aic_base;
static struct irq_domain *aic_domain;

static void nuc970_aic_mask(struct irq_data *d)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);

	if (hw < 32)
		writel_relaxed(BIT(hw), aic_base + AIC_MDCR);
	else
		writel_relaxed(BIT(hw - 32), aic_base + AIC_MDCRH);
}

static void nuc970_aic_unmask(struct irq_data *d)
{
	irq_hw_number_t hw = irqd_to_hwirq(d);

	if (hw < 32)
		writel_relaxed(BIT(hw), aic_base + AIC_MECR);
	else
		writel_relaxed(BIT(hw - 32), aic_base + AIC_MECRH);
}

static void nuc970_aic_ack(struct irq_data *d)
{
	writel_relaxed(1, aic_base + AIC_EOSCR);
}

static struct irq_chip nuc970_aic_chip = {
	.name		= "AIC",
	.irq_ack	= nuc970_aic_ack,
	.irq_mask	= nuc970_aic_mask,
	.irq_unmask	= nuc970_aic_unmask,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static void __exception_irq_entry nuc970_aic_handle_irq(struct pt_regs *regs)
{
	u32 hwirq;

	/* reading IPER updates ISNR and puts the source into service */
	readl_relaxed(aic_base + AIC_IPER);
	hwirq = readl_relaxed(aic_base + AIC_ISNR);

	/* source 0 is reserved: nothing pending */
	if (!hwirq)
		return;

	generic_handle_domain_irq(aic_domain, hwirq);
}

static int nuc970_aic_map(struct irq_domain *d, unsigned int virq,
			  irq_hw_number_t hw)
{
	if (hw == 0 || hw >= AIC_NUM_IRQS)
		return -EINVAL;

	irq_set_chip_and_handler(virq, &nuc970_aic_chip, handle_level_irq);
	irq_set_probe(virq);
	return 0;
}

static const struct irq_domain_ops nuc970_aic_ops = {
	.map	= nuc970_aic_map,
	.xlate	= irq_domain_xlate_twocell,
};

static int __init nuc970_aic_of_init(struct device_node *node,
				     struct device_node *parent)
{
	if (parent)
		return -EINVAL;

	aic_base = of_iomap(node, 0);
	if (!aic_base) {
		pr_err("%pOF: failed to map registers\n", node);
		return -ENOMEM;
	}

	/* mask everything except bits 0-1 (reserved, WDT), as the BSP does */
	writel_relaxed(0xfffffffc, aic_base + AIC_MDCR);
	writel_relaxed(0xffffffff, aic_base + AIC_MDCRH);

	aic_domain = irq_domain_create_linear(of_fwnode_handle(node),
					      AIC_NUM_IRQS, &nuc970_aic_ops,
					      NULL);
	if (!aic_domain) {
		iounmap(aic_base);
		return -ENOMEM;
	}

	set_handle_irq(nuc970_aic_handle_irq);
	return 0;
}
IRQCHIP_DECLARE(nuc970_aic, "nuvoton,nuc970-aic", nuc970_aic_of_init);
