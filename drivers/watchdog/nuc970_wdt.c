// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 watchdog timer
 *
 * Based on the Nuvoton BSP driver (Copyright (c) 2014 Nuvoton Technology
 * corporation). The watchdog counts the 32.768 kHz XIN32K clock and offers
 * three useful intervals: 0.53 s, 2.03 s and 8.03 s. Like the BSP, a
 * requested timeout below 2 s selects 0.53 s, below 8 s selects 2.03 s and
 * anything else 8.03 s, with the timeout capped at 9 s.
 *
 * WDT_CR is protected by the GCR register write-protection lock, so every
 * write unlocks GCR_WRPRTR first and relocks it afterwards.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define WDT_CR			0x00
#define  WDT_CR_WTIS		GENMASK(10, 8)	/* interval select */
#define  WDT_CR_WTE		BIT(7)		/* enable */
#define  WDT_CR_WTRF		BIT(2)		/* reset flag */
#define  WDT_CR_WTRE		BIT(1)		/* reset enable */
#define  WDT_CR_WTR		BIT(0)		/* restart the counter */

#define WDT_WTIS_0_53S		5
#define WDT_WTIS_2_03S		6
#define WDT_WTIS_8_03S		7

#define GCR_RSTSTS		0x6c
#define  GCR_RSTSTS_WDT		BIT(5)
#define GCR_WRPRTR		0x1fc

#define WDT_DEFAULT_TIMEOUT	9
#define WDT_MAX_TIMEOUT		9

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int heartbeat;
module_param(heartbeat, uint, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog timeout in seconds (default="
		 __MODULE_STRING(WDT_DEFAULT_TIMEOUT) ")");

struct nuc970_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	void __iomem *gcr;
	spinlock_t lock;
};

static void nuc970_wdt_write(struct nuc970_wdt *wdt, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	do {
		writel(0x59, wdt->gcr + GCR_WRPRTR);
		writel(0x16, wdt->gcr + GCR_WRPRTR);
		writel(0x88, wdt->gcr + GCR_WRPRTR);
	} while (!(readl(wdt->gcr + GCR_WRPRTR) & 1));
	writel(val, wdt->base + WDT_CR);
	writel(0, wdt->gcr + GCR_WRPRTR);
	spin_unlock_irqrestore(&wdt->lock, flags);
}

static u32 nuc970_wdt_wtis(unsigned int timeout)
{
	if (timeout < 2)
		return WDT_WTIS_0_53S;
	if (timeout < 8)
		return WDT_WTIS_2_03S;
	return WDT_WTIS_8_03S;
}

static int nuc970_wdt_ping(struct watchdog_device *wdd)
{
	struct nuc970_wdt *wdt = watchdog_get_drvdata(wdd);

	/* WTRF is write-one-to-clear; do not clear it by accident */
	nuc970_wdt_write(wdt, (readl(wdt->base + WDT_CR) & ~WDT_CR_WTRF) |
			      WDT_CR_WTR);
	return 0;
}

static int nuc970_wdt_start(struct watchdog_device *wdd)
{
	struct nuc970_wdt *wdt = watchdog_get_drvdata(wdd);

	nuc970_wdt_write(wdt, FIELD_PREP(WDT_CR_WTIS,
					 nuc970_wdt_wtis(wdd->timeout)) |
			      WDT_CR_WTE | WDT_CR_WTRE | WDT_CR_WTR);
	return 0;
}

static int nuc970_wdt_stop(struct watchdog_device *wdd)
{
	struct nuc970_wdt *wdt = watchdog_get_drvdata(wdd);

	nuc970_wdt_write(wdt, 0);
	return 0;
}

static int nuc970_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	wdd->timeout = timeout;
	if (watchdog_active(wdd))
		return nuc970_wdt_start(wdd);
	return 0;
}

static const struct watchdog_info nuc970_wdt_info = {
	.identity	= "nuc970 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops nuc970_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= nuc970_wdt_start,
	.stop		= nuc970_wdt_stop,
	.ping		= nuc970_wdt_ping,
	.set_timeout	= nuc970_wdt_set_timeout,
};

static void nuc970_wdt_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static void nuc970_wdt_iounmap(void *addr)
{
	iounmap(addr);
}

/* The GCR is shared with other users; map it without claiming the region. */
static void __iomem *nuc970_wdt_map_gcr(struct device *dev)
{
	struct device_node *np;
	void __iomem *gcr;

	np = of_parse_phandle(dev->of_node, "nuvoton,gcr", 0);
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "nuvoton,nuc970-gcr");
	if (!np)
		return NULL;

	gcr = of_iomap(np, 0);
	of_node_put(np);
	if (!gcr)
		return NULL;

	if (devm_add_action_or_reset(dev, nuc970_wdt_iounmap, (void __force *)gcr))
		return NULL;
	return gcr;
}

static int nuc970_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_wdt *wdt;
	struct clk *eclk, *lxt, *pclk;
	int err;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	spin_lock_init(&wdt->lock);

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	wdt->gcr = nuc970_wdt_map_gcr(dev);
	if (!wdt->gcr)
		return dev_err_probe(dev, -ENODEV, "cannot map GCR\n");

	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk), "no pclk\n");

	eclk = devm_clk_get(dev, "eclk");
	if (IS_ERR(eclk))
		return dev_err_probe(dev, PTR_ERR(eclk), "no eclk\n");
	lxt = devm_clk_get(dev, "lxt");
	if (IS_ERR(lxt))
		return dev_err_probe(dev, PTR_ERR(lxt), "no lxt\n");

	/* count the 32.768 kHz crystal, as the BSP's XIN32K option does */
	err = clk_set_parent(clk_get_parent(eclk), lxt);
	if (err)
		return dev_err_probe(dev, err, "cannot select XIN32K\n");
	err = clk_prepare_enable(eclk);
	if (err)
		return err;
	err = devm_add_action_or_reset(dev, nuc970_wdt_clk_disable, eclk);
	if (err)
		return err;

	wdt->wdd.info = &nuc970_wdt_info;
	wdt->wdd.ops = &nuc970_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = WDT_DEFAULT_TIMEOUT;
	watchdog_init_timeout(&wdt->wdd, heartbeat, dev);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_set_drvdata(&wdt->wdd, wdt);

	if (readl(wdt->gcr + GCR_RSTSTS) & GCR_RSTSTS_WDT)
		wdt->wdd.bootstatus = WDIOF_CARDRESET;

	/* left running by the boot loader: let the core keep it fed */
	if (readl(wdt->base + WDT_CR) & WDT_CR_WTE) {
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		nuc970_wdt_start(&wdt->wdd);
	}

	err = devm_watchdog_register_device(dev, &wdt->wdd);
	if (err)
		return err;

	dev_info(dev, "watchdog, timeout %u s%s%s\n", wdt->wdd.timeout,
		 nowayout ? ", nowayout" : "",
		 wdt->wdd.bootstatus ? ", last reset by watchdog" : "");
	return 0;
}

static const struct of_device_id nuc970_wdt_of_match[] = {
	{ .compatible = "nuvoton,nuc970-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_wdt_of_match);

static struct platform_driver nuc970_wdt_driver = {
	.probe	= nuc970_wdt_probe,
	.driver	= {
		.name		= "nuc970-wdt",
		.of_match_table	= nuc970_wdt_of_match,
	},
};
module_platform_driver(nuc970_wdt_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 watchdog timer");
MODULE_LICENSE("GPL");
