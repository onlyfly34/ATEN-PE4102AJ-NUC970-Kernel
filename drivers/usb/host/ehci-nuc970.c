// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NUC970/NUC977 EHCI glue
 *
 * Forward-ported from the EHCI glue of Nuvoton's NUC970 Linux BSP (3.10
 * and 5.10, Copyright (c) 2012-2023 Nuvoton Technology Corporation) as a
 * stand-alone glue driver in the style of ehci-npcm7xx.c.
 *
 * Besides the standard EHCI core this powers up the two USB PHYs (vendor
 * registers in the EHCI window) and programs the over-current polarity (a
 * register in the companion OHCI's window), exactly as the BSP did.
 */

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ehci.h"
#include "nuc970-usbh.h"

#define DRIVER_DESC "Nuvoton NUC970 EHCI driver"

static struct hc_driver __read_mostly ehci_nuc970_hc_driver;

static int ehci_nuc970_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *ohci_np = NULL;
	void __iomem *ohci_regs;
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

	hcd = usb_create_hcd(&ehci_nuc970_hc_driver, dev, dev_name(dev));
	if (!hcd)
		return -ENOMEM;

	hcd->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto err_put_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	/* over-current polarity lives in the OHCI register window */
	ohci_regs = nuc970_usbh_map_sibling(dev, NUC970_OHCI_COMPATIBLE,
					    &ohci_np);
	if (IS_ERR(ohci_regs)) {
		dev_warn(dev, "no OHCI node, over-current polarity left as is\n");
	} else {
		oc_low = nuc970_usbh_oc_active_low(dev->of_node, ohci_np);
		nuc970_usbh_set_oc_level(ohci_regs, oc_low);
		of_node_put(ohci_np);
	}

	nuc970_usbh_phy_on(hcd->regs);

	/* capability registers at offset 0, operational ones at CAPLENGTH */
	hcd_to_ehci(hcd)->caps = hcd->regs;

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto err_put_hcd;

	device_wakeup_enable(hcd->self.controller);
	return 0;

err_put_hcd:
	usb_put_hcd(hcd);
	return dev_err_probe(dev, ret, "failed to add EHCI host\n");
}

static void ehci_nuc970_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

/*
 * The root hubs of both controllers are suspended before either platform
 * device, so whichever controller goes first may power the PHYs down, and
 * whichever resumes first powers them up again.
 */
static int ehci_nuc970_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	int ret;

	ret = ehci_suspend(hcd, device_may_wakeup(dev));
	if (ret)
		return ret;

	nuc970_usbh_phy_off(hcd->regs);
	return 0;
}

static int ehci_nuc970_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	nuc970_usbh_phy_on(hcd->regs);
	ehci_resume(hcd, false);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ehci_nuc970_pm_ops, ehci_nuc970_suspend,
				ehci_nuc970_resume);

static const struct of_device_id ehci_nuc970_of_match[] = {
	{ .compatible = NUC970_EHCI_COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, ehci_nuc970_of_match);

static struct platform_driver ehci_nuc970_driver = {
	.probe		= ehci_nuc970_probe,
	.remove		= ehci_nuc970_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name		= "nuc970-ehci",
		.pm		= pm_sleep_ptr(&ehci_nuc970_pm_ops),
		.of_match_table	= ehci_nuc970_of_match,
	},
};

static int __init ehci_nuc970_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ehci_init_driver(&ehci_nuc970_hc_driver, NULL);
	return platform_driver_register(&ehci_nuc970_driver);
}
module_init(ehci_nuc970_init);

static void __exit ehci_nuc970_exit(void)
{
	platform_driver_unregister(&ehci_nuc970_driver);
}
module_exit(ehci_nuc970_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:nuc970-ehci");
MODULE_LICENSE("GPL");
