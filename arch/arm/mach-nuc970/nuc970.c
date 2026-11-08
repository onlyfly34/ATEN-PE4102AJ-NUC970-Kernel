// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 (ARM926EJ-S) device tree platform
 *
 * Forward-ported from arch/arm/mach-nuc970/ in Nuvoton's NUC970 Linux BSP
 * (Copyright (C) 2014 Nuvoton technology corporation). Everything except the
 * system reset and the PE4102 board watchdog keep-alive lives in DT drivers.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/timer.h>

#include <asm/mach/arch.h>

/* GCR (system global control) registers */
#define GCR_AHBIPRST		0x060
#define GCR_MFP_GPB_H		0x07c
#define GCR_WRPRTR		0x1fc

static void __iomem *nuc970_gcr;

static void __iomem *nuc970_map_compatible(const char *compat)
{
	struct device_node *np;
	void __iomem *base;

	np = of_find_compatible_node(NULL, NULL, compat);
	if (!np)
		return NULL;
	base = of_iomap(np, 0);
	of_node_put(np);
	return base;
}

static void nuc970_restart(enum reboot_mode mode, const char *cmd)
{
	if (!nuc970_gcr)
		return;

	/* unlock the write-protected registers */
	while (readl(nuc970_gcr + GCR_WRPRTR) != 1) {
		writel(0x59, nuc970_gcr + GCR_WRPRTR);
		writel(0x16, nuc970_gcr + GCR_WRPRTR);
		writel(0x88, nuc970_gcr + GCR_WRPRTR);
	}
	writel(1, nuc970_gcr + GCR_AHBIPRST);	/* CHIPRST */
}

/*
 * ATEN PE4102 external hardware watchdog ("WDI") keep-alive.
 *
 * The board's external watchdog IC is fed through GPIO PB12 (legacy GPIO 44).
 * If PB12 stops toggling the IC hard-resets the board about 3 s into boot,
 * long before userland runs. Like the vendor kernel and our 5.10 port, hold
 * the GPIO APB clock on, put PB12 in GPIO output mode and toggle it from a
 * kernel timer. A real lockup stops the timer, so the external watchdog
 * still catches hangs. The pin is driven directly (not through gpiolib) so
 * that userland can still export GPIO 44 if it wants to.
 */
#define WDI_GPIO_DIR		0x40		/* GPIOB_DIR     */
#define WDI_GPIO_DOUT		0x44		/* GPIOB_DATAOUT */
#define WDI_BIT			BIT(12)		/* PB12          */
#define WDI_PING_MS		100

static void __iomem *nuc970_wdi_gpio;
static struct timer_list nuc970_wdi_timer;

static void nuc970_wdi_ping(struct timer_list *unused)
{
	writel(readl(nuc970_wdi_gpio + WDI_GPIO_DOUT) ^ WDI_BIT,
	       nuc970_wdi_gpio + WDI_GPIO_DOUT);
	mod_timer(&nuc970_wdi_timer, jiffies + msecs_to_jiffies(WDI_PING_MS));
}

static int __init nuc970_wdi_init(void)
{
	struct device_node *np;
	struct clk *clk;
	u32 v;

	if (!of_machine_is_compatible("aten,pe4102"))
		return 0;

	np = of_find_compatible_node(NULL, NULL, "nuvoton,nuc970-gpio");
	if (!np) {
		pr_err("nuc970 wdi: no GPIO node\n");
		return -ENODEV;
	}
	nuc970_wdi_gpio = of_iomap(np, 0);
	/* keep the GPIO APB clock enabled for the lifetime of the system */
	clk = of_clk_get_by_name(np, "pclk");
	of_node_put(np);
	if (!nuc970_wdi_gpio || !nuc970_gcr) {
		pr_err("nuc970 wdi: cannot map GPIO/GCR\n");
		return -ENOMEM;
	}
	if (IS_ERR(clk) || clk_prepare_enable(clk))
		pr_err("nuc970 wdi: cannot enable GPIO clock\n");

	/* PB12 -> GPIO function (MFP_GPB_H[19:16] = 0), output */
	v = readl(nuc970_gcr + GCR_MFP_GPB_H);
	writel(v & ~(0xfu << 16), nuc970_gcr + GCR_MFP_GPB_H);
	writel(readl(nuc970_wdi_gpio + WDI_GPIO_DIR) | WDI_BIT,
	       nuc970_wdi_gpio + WDI_GPIO_DIR);

	pr_info("nuc970 wdi: feeding external watchdog on PB12 every %d ms\n",
		WDI_PING_MS);
	timer_setup(&nuc970_wdi_timer, nuc970_wdi_ping, 0);
	mod_timer(&nuc970_wdi_timer, jiffies + msecs_to_jiffies(WDI_PING_MS));
	return 0;
}

static int __init nuc970_early_init(void)
{
	if (!of_machine_is_compatible("nuvoton,nuc970"))
		return 0;

	nuc970_gcr = nuc970_map_compatible("nuvoton,nuc970-gcr");
	if (!nuc970_gcr)
		pr_warn("nuc970: no GCR node, restart unavailable\n");

	return nuc970_wdi_init();
}
early_initcall(nuc970_early_init);

static const char *const nuc970_dt_compat[] __initconst = {
	"nuvoton,nuc970",
	NULL
};

DT_MACHINE_START(NUC970_DT, "Nuvoton NUC970 (Device Tree)")
	.dt_compat	= nuc970_dt_compat,
	.restart	= nuc970_restart,
MACHINE_END
