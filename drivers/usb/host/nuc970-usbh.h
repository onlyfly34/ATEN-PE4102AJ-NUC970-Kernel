/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Nuvoton NUC970/NUC977 USB host: settings shared by the EHCI and OHCI glue
 *
 * The two controllers share the PHYs, the over-current input and the
 * USBH/USB engine clock gate.  The PHY controls live in the EHCI register
 * window and the over-current polarity lives in the OHCI window, and the
 * Nuvoton BSP (drivers/usb/host/{ehci,ohci}-nuc970.c, Copyright (c) 2012-2023
 * Nuvoton Technology Corporation) programs all of them from both drivers so
 * that either controller works on its own.  This header keeps that
 * behaviour.
 */

#ifndef __USB_HOST_NUC970_USBH_H
#define __USB_HOST_NUC970_USBH_H

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define NUC970_EHCI_COMPATIBLE	"nuvoton,nuc970-ehci"
#define NUC970_OHCI_COMPATIBLE	"nuvoton,nuc970-ohci"

/* EHCI window: USB PHY0/PHY1 control */
#define NUC970_EHCI_USBPCR0	0xc4
#define NUC970_EHCI_USBPCR1	0xc8
#define NUC970_USBPCR0_ON	0x160	/* values written by the BSP */
#define NUC970_USBPCR1_ON	0x520
#define NUC970_USBPCR0_OFF	0x060	/* PHY suspended, powered down */
#define NUC970_USBPCR1_OFF	0x020

/* OHCI window: miscellaneous control */
#define NUC970_OHCI_HCMISC	0x204
#define NUC970_HCMISC_OCAL	BIT(3)	/* over-current input active low */

/* the OHCI engine needs 48 MHz (BSP: APLL 96 MHz / 2) */
#define NUC970_USB_ECLK_RATE	48000000UL

static inline void nuc970_usbh_phy_on(void __iomem *ehci)
{
	writel(NUC970_USBPCR0_ON, ehci + NUC970_EHCI_USBPCR0);
	writel(NUC970_USBPCR1_ON, ehci + NUC970_EHCI_USBPCR1);
}

static inline void nuc970_usbh_phy_off(void __iomem *ehci)
{
	writel(NUC970_USBPCR0_OFF, ehci + NUC970_EHCI_USBPCR0);
	writel(NUC970_USBPCR1_OFF, ehci + NUC970_EHCI_USBPCR1);
}

static inline void nuc970_usbh_set_oc_level(void __iomem *ohci, bool low)
{
	u32 val = readl(ohci + NUC970_OHCI_HCMISC);

	if (low)
		val |= NUC970_HCMISC_OCAL;
	else
		val &= ~NUC970_HCMISC_OCAL;
	writel(val, ohci + NUC970_OHCI_HCMISC);
}

/*
 * Map the register window of the sibling controller.  It is only ioremapped,
 * not requested, because the sibling's own driver requests it.
 * @np_out (optional) returns a reference to the sibling's node.
 */
static inline void __iomem *
nuc970_usbh_map_sibling(struct device *dev, const char *compatible,
			struct device_node **np_out)
{
	struct device_node *np;
	struct resource res;
	void __iomem *base;
	int ret;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
		return IOMEM_ERR_PTR(-ENODEV);

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		of_node_put(np);
		return IOMEM_ERR_PTR(ret);
	}

	base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!base) {
		of_node_put(np);
		return IOMEM_ERR_PTR(-ENOMEM);
	}

	if (np_out)
		*np_out = np;
	else
		of_node_put(np);
	return base;
}

/*
 * "nuvoton,oc-active-low" belongs to the OHCI node (the register is in its
 * window); it is also honoured on the EHCI node.  Without it the input is
 * active high, which is what the BSP uses when no over-current pin is muxed.
 */
static inline bool nuc970_usbh_oc_active_low(const struct device_node *a,
					     const struct device_node *b)
{
	return (a && of_property_read_bool(a, "nuvoton,oc-active-low")) ||
	       (b && of_property_read_bool(b, "nuvoton,oc-active-low"));
}

/*
 * Claim the AHB clock and the engine clock.  Both are gated by HCLKEN bit 18,
 * so both must be held or clk_disable_unused() turns the block off.
 */
static inline int nuc970_usbh_clk_init(struct device *dev)
{
	struct clk *hclk, *eclk;
	unsigned long rate;

	hclk = devm_clk_get_enabled(dev, "hclk");
	if (IS_ERR(hclk))
		return dev_err_probe(dev, PTR_ERR(hclk),
				     "failed to enable hclk\n");

	eclk = devm_clk_get_enabled(dev, "eclk");
	if (IS_ERR(eclk))
		return dev_err_probe(dev, PTR_ERR(eclk),
				     "failed to enable eclk\n");

	rate = clk_get_rate(eclk);
	if (rate != NUC970_USB_ECLK_RATE &&
	    !clk_set_rate(eclk, NUC970_USB_ECLK_RATE))
		rate = clk_get_rate(eclk);

	/* full speed allows +-0.25 % */
	if (abs_diff(rate, NUC970_USB_ECLK_RATE) > NUC970_USB_ECLK_RATE / 400)
		dev_warn(dev,
			 "USB engine clock is %lu Hz, not 48 MHz; full/low-speed devices will not work\n",
			 rate);

	return 0;
}

#endif /* __USB_HOST_NUC970_USBH_H */
