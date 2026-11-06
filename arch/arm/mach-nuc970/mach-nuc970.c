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
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include "cpu.h"
#include "pm.h"

/*
 * External hardware-watchdog "WDI" keep-alive (replaces ATEN's out-of-tree
 * "initWdt"/"timerCallBack").  The PE4102 board carries an EXTERNAL watchdog IC
 * (independent of the SoC WDT/WWDT, both of which U-Boot leaves disabled); its
 * WDI input is wired to NUC970 GPIO 44 = PB12.  If PB12 is not toggled, the
 * external IC hard-resets the board ~3s into boot, before userland's watchdog
 * service starts.  Reverse-engineered from the vendor 1.0.069 kernel: initWdt()
 * does gpio_request(44) + direction_output + sets up a 350ms timer whose
 * handler timerCallBack() toggles GPIO 44 each tick.  We replicate it with
 * direct register access (the GPIO platform driver is a late device_initcall,
 * too late vs the 3s external-WDT window): ungate the "gpio" pclk, put PB12 in
 * GPIO-output mode, and toggle PB12 every 300ms from an early_initcall timer.
 * A real kernel lockup stops the timer, so the external WDT still protects
 * against hangs.  Clocks are registered in time_init(), before any initcall.
 *
 * GPIO 44: GPIO_OFFSET=0x20, so group = 44/32 = 1 (port B), bit = 44%32 = 12.
 * PB12 MFP nibble lives in REG_MFP_GPB_H bits [19:16] (0 = GPIO).
 */
#define WDI_DIR		REG_GPIOB_DIR		/* GPIO_BA + 0x40 */
#define WDI_DOUT	REG_GPIOB_DATAOUT	/* GPIO_BA + 0x44 */
#define WDI_BIT		(1u << 12)		/* PB12            */
#define WDI_PING_MS	300
static struct timer_list nuc970_wdi_timer;

static void nuc970_wdi_fn(unsigned long data)
{
	__raw_writel(__raw_readl(WDI_DOUT) ^ WDI_BIT, WDI_DOUT);	/* toggle PB12 */
	mod_timer(&nuc970_wdi_timer,
		  jiffies + msecs_to_jiffies(WDI_PING_MS));
}

static int __init nuc970_wdi_init(void)
{
	struct clk *clk, *eclk;
	u32 v;

	/* ungate the GPIO APB clock(s) so the port registers are accessible */
	clk = clk_get(NULL, "gpio");
	if (!IS_ERR(clk))
		clk_prepare_enable(clk);
	eclk = clk_get(NULL, "gpio_eclk");
	if (!IS_ERR(eclk))
		clk_prepare_enable(eclk);

	/* PB12 -> GPIO function (MFP nibble bits [19:16] = 0) */
	v = __raw_readl(REG_MFP_GPB_H);
	v &= ~(0xFu << 16);
	__raw_writel(v, REG_MFP_GPB_H);

	/* PB12 -> output (DIR bit = 1 means output on NUC970) */
	__raw_writel(__raw_readl(WDI_DIR) | WDI_BIT, WDI_DIR);

	pr_info("nuc970 wdi: WDI=PB12 (gpio44) toggle %dms\n", WDI_PING_MS);
	setup_timer(&nuc970_wdi_timer, nuc970_wdi_fn, 0);
	mod_timer(&nuc970_wdi_timer,
		  jiffies + msecs_to_jiffies(WDI_PING_MS));
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

extern void nuc970_restart(char mode, const char *cmd);
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

static void __init nuc970_reserve_memory(void)
{
	if(memblock_reserve(0, 1024) < 0)
		printk("Failed to reserve memory 0x0~0x400\n");
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
