// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 timer
 *
 * Forward-ported from arch/arm/mach-nuc970/time.c in Nuvoton's NUC970 Linux
 * BSP (Copyright (c) 2014 Nuvoton technology corporation).
 *
 * TIMER0 is the clock event device, TIMER1 a free-running 24-bit up counter
 * used as clocksource and sched_clock. Both are clocked from XIN (12 MHz).
 * TIMER0 divides it by 100 as in the BSP. TIMER1 counts at the full 12 MHz:
 * the ARM926 has no cycle counter, so random_get_entropy() falls back to
 * this clocksource. At the BSP's 120 kHz consecutive reads rarely differ,
 * the CRNG's timing-jitter entropy source gives up, and getrandom() or
 * /dev/random block for minutes after boot until interrupt entropy alone
 * seeds the CRNG. The 24-bit counter then wraps every 1.4 s, which
 * sched_clock and the timekeeping core handle.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define TMR_TCSR(n)	(0x10 * (n) + 0x00)
#define TMR_TICR(n)	(0x10 * (n) + 0x04)
#define TMR_TDR(n)	(0x10 * (n) + 0x08)
#define TMR_TISR	0x60

#define TCSR_CEN	BIT(30)
#define TCSR_IE		BIT(29)
#define TCSR_MODE_MASK	(0x3 << 27)
#define TCSR_ONESHOT	(0x0 << 27)
#define TCSR_PERIODIC	(0x1 << 27)
#define TCSR_PRESCALE	0x63		/* TIMER0: divide by prescale + 1 = 100 */
#define TCSR_PRESCALE1	0x00		/* TIMER1: undivided */

#define TDR_BITS	24

static void __iomem *tmr_base;
static u32 tmr_periodic_load;

static int nuc970_tmr_shutdown(struct clock_event_device *evt)
{
	writel_relaxed(0, tmr_base + TMR_TCSR(0));
	return 0;
}

static int nuc970_tmr_set_periodic(struct clock_event_device *evt)
{
	u32 val = readl_relaxed(tmr_base + TMR_TCSR(0)) & ~TCSR_MODE_MASK;

	writel_relaxed(tmr_periodic_load, tmr_base + TMR_TICR(0));
	writel_relaxed(val | TCSR_PERIODIC | TCSR_CEN | TCSR_IE |
		       TCSR_PRESCALE, tmr_base + TMR_TCSR(0));
	return 0;
}

static int nuc970_tmr_set_oneshot(struct clock_event_device *evt)
{
	u32 val = readl_relaxed(tmr_base + TMR_TCSR(0)) & ~TCSR_MODE_MASK;

	writel_relaxed(val | TCSR_ONESHOT | TCSR_CEN | TCSR_IE |
		       TCSR_PRESCALE, tmr_base + TMR_TCSR(0));
	return 0;
}

/* Same sequence as the BSP: reload TICR, restart the counter if it stopped. */
static int nuc970_tmr_set_next_event(unsigned long cycles,
				     struct clock_event_device *evt)
{
	u32 tcsr = readl_relaxed(tmr_base + TMR_TCSR(0));
	u32 delta = readl_relaxed(tmr_base + TMR_TICR(0)) -
		    readl_relaxed(tmr_base + TMR_TDR(0));

	writel_relaxed(cycles, tmr_base + TMR_TICR(0));
	if (!(tcsr & TCSR_CEN) && (delta > 2 || delta == 0))
		writel_relaxed(readl_relaxed(tmr_base + TMR_TCSR(0)) | TCSR_CEN,
			       tmr_base + TMR_TCSR(0));
	return 0;
}

static struct clock_event_device nuc970_clockevent = {
	.name			= "nuc970-timer0",
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 300,
	.set_state_shutdown	= nuc970_tmr_shutdown,
	.set_state_periodic	= nuc970_tmr_set_periodic,
	.set_state_oneshot	= nuc970_tmr_set_oneshot,
	.tick_resume		= nuc970_tmr_shutdown,
	.set_next_event		= nuc970_tmr_set_next_event,
};

static irqreturn_t nuc970_tmr_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	writel_relaxed(BIT(0), tmr_base + TMR_TISR);	/* clear TIF0 */
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static u64 notrace nuc970_read_sched_clock(void)
{
	return readl_relaxed(tmr_base + TMR_TDR(1)) & GENMASK(TDR_BITS - 1, 0);
}

static unsigned long __init nuc970_tmr_enable_clk(struct device_node *np,
						  int index)
{
	struct clk *clk = of_clk_get(np, index);

	if (IS_ERR(clk) || clk_prepare_enable(clk)) {
		pr_err("%pOF: failed to enable clock %d\n", np, index);
		return 0;
	}
	return clk_get_rate(clk);
}

static int __init nuc970_timer_init(struct device_node *np)
{
	unsigned long rate0, rate1;
	int irq, ret;

	tmr_base = of_iomap(np, 0);
	if (!tmr_base) {
		pr_err("%pOF: failed to map registers\n", np);
		return -ENXIO;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%pOF: failed to map interrupt\n", np);
		ret = -EINVAL;
		goto err_unmap;
	}

	rate0 = nuc970_tmr_enable_clk(np, 0) / (TCSR_PRESCALE + 1);
	rate1 = nuc970_tmr_enable_clk(np, 1) / (TCSR_PRESCALE1 + 1);
	if (!rate0 || !rate1) {
		ret = -EINVAL;
		goto err_unmap;
	}

	/* TIMER1: free running clocksource */
	writel_relaxed(0, tmr_base + TMR_TCSR(1));
	writel_relaxed(0xffffffff, tmr_base + TMR_TICR(1));
	writel_relaxed(TCSR_CEN | TCSR_PERIODIC | TCSR_PRESCALE1,
		       tmr_base + TMR_TCSR(1));

	ret = clocksource_mmio_init(tmr_base + TMR_TDR(1), "nuc970-timer1",
				    rate1, 200, TDR_BITS,
				    clocksource_mmio_readl_up);
	if (ret) {
		pr_err("%pOF: failed to register clocksource\n", np);
		goto err_unmap;
	}
	sched_clock_register(nuc970_read_sched_clock, TDR_BITS, rate1);

	/* TIMER0: clock events */
	writel_relaxed(0, tmr_base + TMR_TCSR(0));
	writel_relaxed(0x1f, tmr_base + TMR_TISR);
	tmr_periodic_load = DIV_ROUND_CLOSEST(rate0, HZ) - 1;

	ret = request_irq(irq, nuc970_tmr_interrupt, IRQF_TIMER | IRQF_IRQPOLL,
			  "timer0", &nuc970_clockevent);
	if (ret) {
		pr_err("%pOF: failed to request irq %d\n", np, irq);
		goto err_unmap;
	}

	nuc970_clockevent.cpumask = cpumask_of(0);
	clockevents_config_and_register(&nuc970_clockevent, rate0, 12,
					GENMASK(TDR_BITS - 1, 0));
	return 0;

err_unmap:
	iounmap(tmr_base);
	return ret;
}
TIMER_OF_DECLARE(nuc970, "nuvoton,nuc970-timer", nuc970_timer_init);
