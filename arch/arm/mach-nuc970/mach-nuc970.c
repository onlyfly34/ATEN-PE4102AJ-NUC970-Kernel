 /*
 * linux/arch/arm/mach-nuc970/mach-nuc970.c
 *
 * Copyright (C) 2014 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "cpu.h"
#include "pm.h"

/*
 * External hardware-watchdog "WDI" keep-alive (PE4102 board). Its WDI input is
 * NUC970 GPIO44 = PB12; if PB12 is not toggled the external IC hard-resets the
 * board ~3s into boot, before userland. Replicate the vendor initWdt: hold the
 * GPIO APB clock on (via the clock framework), put PB12 in GPIO-output mode, and
 * toggle it every WDI_PING_MS from an early_initcall timer. A real lockup stops
 * the timer so the external WDT still protects against hangs.
 *
 * Clock handling (5.10): the GPIO APB gate is REG_CLK_PCLKEN0 bit3, registered by
 * nuc970_init_clocks() as clkdev "gpio". That runs from time_init() (before any
 * initcall), so clk_get(NULL,"gpio") here at early_initcall succeeds; a held
 * clk_prepare_enable() keeps the gate on for the board's lifetime, so
 * clk_disable_unused() (late_initcall) cannot gate it and no per-tick re-assert is
 * needed. (The gpio-nuc970 driver also clk_enable()s "gpio" at its device_initcall
 * probe and does not reset PB12's MFP/DIR, so MFP+DIR are set once here too.)
 */
#define WDI_DIR		REG_GPIOB_DIR		/* GPIO_BA + 0x40 */
#define WDI_DOUT	REG_GPIOB_DATAOUT	/* GPIO_BA + 0x44 */
#define WDI_BIT		(1u << 12)		/* PB12            */
#define WDI_PING_MS	100
static struct timer_list nuc970_wdi_timer;
static struct clk *nuc970_wdi_clk;	/* held enabled for the board's lifetime */

static void nuc970_wdi_fn(struct timer_list *unused)
{
	/* Just feed the external watchdog: toggle PB12. The GPIO APB clock is held on
	 * via the clock framework and nothing re-muxes/re-directions PB12, so no
	 * per-tick clock/MFP/DIR re-assert is needed. */
	__raw_writel(__raw_readl(WDI_DOUT) ^ WDI_BIT, WDI_DOUT);	/* toggle PB12 */
	mod_timer(&nuc970_wdi_timer, jiffies + msecs_to_jiffies(WDI_PING_MS));
}

static int __init nuc970_wdi_init(void)
{
	u32 v;

	/* Hold the GPIO APB clock on through the clock framework. */
	nuc970_wdi_clk = clk_get(NULL, "gpio");
	if (!IS_ERR(nuc970_wdi_clk)) {
		clk_prepare_enable(nuc970_wdi_clk);
	} else {
		/* Fallback only (framework unexpectedly not ready): ungate directly so
		 * the WDI still works; without a held ref clk_disable_unused() could
		 * gate it, so warn loudly if we ever land here. */
		__raw_writel(__raw_readl(REG_CLK_PCLKEN0) | (1u << 3), REG_CLK_PCLKEN0);
		nuc970_wdi_clk = NULL;
		pr_warn("nuc970 wdi: clk_get(gpio) failed, fell back to direct PCLKEN0\n");
	}

	/* PB12 -> GPIO function (MFP_GPB_H nibble [19:16] = 0), output. Set once. */
	v = __raw_readl(REG_MFP_GPB_H);
	v &= ~(0xFu << 16);
	__raw_writel(v, REG_MFP_GPB_H);
	__raw_writel(__raw_readl(WDI_DIR) | WDI_BIT, WDI_DIR);

	pr_info("nuc970 wdi: WDI=PB12 (gpio44) toggle %dms, clk=%s\n",
		WDI_PING_MS, nuc970_wdi_clk ? "framework" : "direct-fallback");
	timer_setup(&nuc970_wdi_timer, nuc970_wdi_fn, 0);
	mod_timer(&nuc970_wdi_timer, jiffies + msecs_to_jiffies(WDI_PING_MS));
	return 0;
}
early_initcall(nuc970_wdi_init);

/* Initial IO mappings */	//TODO: MAP ENABLED IP ONLY
static struct map_desc nuc970_iodesc[] __initdata = {
	IODESC_ENT(IRQ),
	IODESC_ENT(GCR_CLK),
	IODESC_ENT(EBI_SDIC),
        IODESC_ENT(EMAC0),
        IODESC_ENT(EMAC1),
        IODESC_ENT(GDMA),
        IODESC_ENT(EHCI),
        IODESC_ENT(OHCI),
        IODESC_ENT(USBDEV),
        IODESC_ENT(LCD),
        IODESC_ENT(ACTL),
        IODESC_ENT(JPEG),
        IODESC_ENT(GE),
        IODESC_ENT(SDH),
        IODESC_ENT(FMI),
        IODESC_ENT(CAP),
        IODESC_ENT(CRYPTO),
        IODESC_ENT(UART),
	IODESC_ENT(TIMER_ETIMER_WDT_WWDT),
        IODESC_ENT(GPIO),
        IODESC_ENT(RTC),
        IODESC_ENT(SC),
	IODESC_ENT(I2C_SPI),
        IODESC_ENT(PWM),
        IODESC_ENT(KPI),
        IODESC_ENT(ADC),
        IODESC_ENT(CAN),
        IODESC_ENT(MTP),
        IODESC_ENT(SRAM),
};

extern void nuc970_restart(enum reboot_mode mode, const char *cmd);
extern void nuc970_timer_init(void);
static struct platform_device *nuc970_dev[] __initdata = {

};

void __init nuc970_map_io(void)
{
	iotable_init(nuc970_iodesc, ARRAY_SIZE(nuc970_iodesc));
}

static void __init nuc970_init(void)
{
	nuc970_platform_init(nuc970_dev, ARRAY_SIZE(nuc970_dev));
}

static void __init nuc970_init_late(void)
{
	nuc970_init_suspend();
}

void __init nuc970_reserve_memory(void)
{
	if (memblock_reserve(0, PAGE_SIZE) < 0)
		pr_err("Failed to reserve memory 0x0~0x%lx\n",
		       PAGE_SIZE);
}

#ifdef CONFIG_CPU_NUC970
MACHINE_START(NUC970, "NUC970")
#elif defined (CONFIG_CPU_N9H30)
MACHINE_START(NUC970, "N9H30")
#endif
	.atag_offset	= 0x100,
	.map_io		= nuc970_map_io,
	.init_irq	= nuc970_init_irq,
	.init_machine	= nuc970_init,
	.init_time	= nuc970_timer_init,
	.init_late	= nuc970_init_late,
	.reserve	= nuc970_reserve_memory,
	.restart	= nuc970_restart,
MACHINE_END
