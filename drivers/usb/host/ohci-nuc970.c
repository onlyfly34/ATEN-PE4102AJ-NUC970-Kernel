// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NUC970/NUC977 OHCI glue
 *
 * Forward-ported from the OHCI glue of Nuvoton's NUC970 Linux BSP (3.10
 * and 5.10, Copyright (c) 2012-2023 Nuvoton Technology Corporation) as a
 * stand-alone glue driver in the style of ohci-spear.c.
 *
 * Besides the standard OHCI core this programs the over-current polarity,
 * powers up the USB PHYs (vendor registers in the companion EHCI's window)
 * and asks for the 48 MHz USB engine clock, as the BSP did.
 */

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"
#include "nuc970-usbh.h"

#define DRIVER_DESC "Nuvoton NUC970 OHCI driver"

struct nuc970_ohci {
	void __iomem *ehci_regs;	/* PHY controls */
};

#define to_nuc970_ohci(hcd)	((struct nuc970_ohci *)hcd_to_ohci(hcd)->priv)

static struct hc_driver __read_mostly ohci_nuc970_hc_driver;

static int ohci_nuc970_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *ehci_np = NULL;
	void __iomem *ehci_regs;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq, ret;
	bool oc_low;

	if (usb_disabled())
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = nuc970_usbh_clk_init(dev);
	if (ret)
		return ret;

	/* the PHY controls live in the EHCI register window */
	ehci_regs = nuc970_usbh_map_sibling(dev, NUC970_EHCI_COMPATIBLE,
					    &ehci_np);
	if (IS_ERR(ehci_regs))
		return dev_err_probe(dev, PTR_ERR(ehci_regs),
				     "cannot map the EHCI window (PHY control)\n");

	hcd = usb_create_hcd(&ohci_nuc970_hc_driver, dev, dev_name(dev));
	if (!hcd) {
		of_node_put(ehci_np);
		return -ENOMEM;
	}

	hcd->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hcd->regs)) {
		of_node_put(ehci_np);
		ret = PTR_ERR(hcd->regs);
		goto err_put_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	to_nuc970_ohci(hcd)->ehci_regs = ehci_regs;

	oc_low = nuc970_usbh_oc_active_low(dev->of_node, ehci_np);
	nuc970_usbh_set_oc_level(hcd->regs, oc_low);
	of_node_put(ehci_np);

	nuc970_usbh_phy_on(ehci_regs);

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto err_put_hcd;

	device_wakeup_enable(hcd->self.controller);
	return 0;

err_put_hcd:
	usb_put_hcd(hcd);
	return dev_err_probe(dev, ret, "failed to add OHCI host\n");
}

static void ohci_nuc970_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

/* see ehci-nuc970.c for why both glue drivers may switch the PHYs */
static int ohci_nuc970_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	int ret;

	ret = ohci_suspend(hcd, device_may_wakeup(dev));
	if (ret)
		return ret;

	nuc970_usbh_phy_off(to_nuc970_ohci(hcd)->ehci_regs);
	return 0;
}

static int ohci_nuc970_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	nuc970_usbh_phy_on(to_nuc970_ohci(hcd)->ehci_regs);
	ohci_resume(hcd, false);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ohci_nuc970_pm_ops, ohci_nuc970_suspend,
				ohci_nuc970_resume);

static const struct of_device_id ohci_nuc970_of_match[] = {
	{ .compatible = NUC970_OHCI_COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, ohci_nuc970_of_match);

static struct platform_driver ohci_nuc970_driver = {
	.probe		= ohci_nuc970_probe,
	.remove		= ohci_nuc970_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name		= "nuc970-ohci",
		.pm		= pm_sleep_ptr(&ohci_nuc970_pm_ops),
		.of_match_table	= ohci_nuc970_of_match,
	},
};

static const struct ohci_driver_overrides ohci_nuc970_overrides __initconst = {
	.product_desc		= "Nuvoton NUC970 OHCI Host Controller",
	.extra_priv_size	= sizeof(struct nuc970_ohci),
};

static int __init ohci_nuc970_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ohci_init_driver(&ohci_nuc970_hc_driver, &ohci_nuc970_overrides);
	return platform_driver_register(&ohci_nuc970_driver);
}
module_init(ohci_nuc970_init);

static void __exit ohci_nuc970_exit(void)
{
	platform_driver_unregister(&ohci_nuc970_driver);
}
module_exit(ohci_nuc970_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:nuc970-ohci");
MODULE_LICENSE("GPL");
