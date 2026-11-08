// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 real time clock
 *
 * Rewritten from drivers/rtc/rtc-nuc970.c in Nuvoton's NUC970 Linux BSP
 * (Copyright (c) 2014 Nuvoton technology corporation) for the current RTC
 * core: DT resources and clock, devm_rtc_allocate_device() with a fixed
 * 2000..2099 range, no hard-coded register mappings.
 *
 * Time and calendar are kept in BCD (TLR/CLR, alarm in TAR/CAR). Registers
 * other than the read-only ones only accept writes after the access-enable
 * sequence: write the INIR magic, then the AER key, and wait for AER[16].
 * Every write is followed by a wait for RIIR[31] (write in progress) to
 * clear. Both sequences are kept as in the BSP.
 *
 * The vendor userland (built against the 3.10 BSP) also uses four private
 * ioctls on /dev/rtc0, which are kept binary compatible:
 *   0x80047020  get the 16 spare registers (64 bytes)
 *   0x40047021  set the 16 spare registers (64 bytes)
 *   0x80047022  read GCR RSTSTS (reset source) into a u32
 *   0x40047023  clear GCR RSTSTS (write back what is set; argument ignored)
 * The last two are a vendor addition found in the shipped 3.10 kernel only;
 * libsys_por_api.so uses them. They need the GCR, which is mapped through
 * the "nuvoton,gcr" phandle (or the "nuvoton,nuc970-gcr" node as fallback).
 *
 * The BSP also enabled the 1 Hz tick interrupt unconditionally and reported
 * it as RTC_UF; the RTC core emulates update interrupts with the alarm
 * timer, so the tick interrupt is left disabled here.
 */

#include <linux/bcd.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/rtc.h>
#include <linux/uaccess.h>

/* RTC registers */
#define RTC_INIR		0x00	/* initiation */
#define RTC_AER			0x04	/* access enable */
#define RTC_FCR			0x08	/* frequency compensation */
#define RTC_TLR			0x0c	/* time loading */
#define RTC_CLR			0x10	/* calendar loading */
#define RTC_TSSR		0x14	/* time scale selection */
#define RTC_DWR			0x18	/* day of the week */
#define RTC_TAR			0x1c	/* time alarm */
#define RTC_CAR			0x20	/* calendar alarm */
#define RTC_LIR			0x24	/* leap year indicator */
#define RTC_RIER		0x28	/* interrupt enable */
#define RTC_RIIR		0x2c	/* interrupt indicator, write 1 to clear */
#define RTC_TTR			0x30	/* time tick */
#define RTC_PWRCTL		0x34	/* power control */
#define RTC_SPR(n)		(0x40 + (n) * 4)	/* spare registers */

#define RTC_INIR_MAGIC		0xa5eb1357
#define RTC_AER_KEY		0xa965
#define RTC_AER_ENABLED		BIT(16)
#define RTC_TSSR_24H		BIT(0)
#define RTC_INT_ALARM		BIT(0)	/* RIER / RIIR */
#define RTC_INT_TICK		BIT(1)	/* RIER / RIIR */
#define RTC_RIIR_BUSY		BIT(31)	/* register write in progress */
#define RTC_CAR_NO_WDAY		BIT(31)	/* do not compare the day of week */
#define RTC_PWRCTL_ALARM	BIT(3)

#define RTC_NUM_SPARE		16

/* GCR registers used by the vendor ioctls and wake-up */
#define GCR_WKUPSER		0x58
#define GCR_WKUPSER_RTC		BIT(24)
#define GCR_RSTSTS		0x6c

/* BSP / vendor private ioctls; the numbers are part of the userland ABI */
#define NUC970_RTC_GET_SPARE_DATA	_IOR('p', 0x20, u32)
#define NUC970_RTC_SET_SPARE_DATA	_IOW('p', 0x21, u32)
#define NUC970_SYS_GET_RSTSTS		_IOR('p', 0x22, u32)
#define NUC970_SYS_RESET_RSTSTS		_IOW('p', 0x23, u32)

static_assert(NUC970_RTC_GET_SPARE_DATA == 0x80047020);
static_assert(NUC970_RTC_SET_SPARE_DATA == 0x40047021);
static_assert(NUC970_SYS_GET_RSTSTS == 0x80047022);
static_assert(NUC970_SYS_RESET_RSTSTS == 0x40047023);

struct nuc970_rtc {
	void __iomem *base;
	void __iomem *gcr;
	struct rtc_device *rtc;
};

static void nuc970_rtc_wait_idle(struct nuc970_rtc *p)
{
	u32 st;

	readl_poll_timeout_atomic(p->base + RTC_RIIR, st, !(st & RTC_RIIR_BUSY),
				  1, 1024);
}

/*
 * Register write followed by the BSP's wait for RIIR[31] (~1 ms max).
 * The extra wait before the write covers the alarm interrupt landing
 * between another write and its completion poll (the BSP had that race):
 * the handler's RIIR write then no longer hits a write in progress.
 */
static void nuc970_rtc_write(struct nuc970_rtc *p, unsigned int reg, u32 val)
{
	nuc970_rtc_wait_idle(p);
	writel(val, p->base + reg);
	nuc970_rtc_wait_idle(p);
}

static void nuc970_rtc_rmw(struct nuc970_rtc *p, unsigned int reg, u32 clr,
			   u32 set)
{
	nuc970_rtc_write(p, reg, (readl(p->base + reg) & ~clr) | set);
}

/*
 * Unlock register writes: INIR magic, 10 ms, AER key, then wait up to
 * 2048 ms for AER[16]. Process context only (all callers hold ops_lock).
 */
static int nuc970_rtc_access_enable(struct nuc970_rtc *p)
{
	u32 st;

	nuc970_rtc_write(p, RTC_INIR, RTC_INIR_MAGIC);
	usleep_range(10000, 11000);
	nuc970_rtc_write(p, RTC_AER, RTC_AER_KEY);

	/*
	 * The BSP meant to return -EPERM here, but its post-decremented
	 * counter made a real timeout fall through as success (the shipped
	 * vendor kernel does the same). Fail instead of writing blind.
	 */
	if (readl_poll_timeout(p->base + RTC_AER, st, st & RTC_AER_ENABLED,
			       1000, 0x800 * 1000))
		return -EPERM;

	return 0;
}

static void nuc970_rtc_bcd2tm(u32 time, u32 cal, struct rtc_time *tm)
{
	tm->tm_sec = bcd2bin(time & 0x7f);
	tm->tm_min = bcd2bin((time >> 8) & 0x7f);
	tm->tm_hour = bcd2bin((time >> 16) & 0x3f);
	tm->tm_mday = bcd2bin(cal & 0x3f);
	tm->tm_mon = bcd2bin((cal >> 8) & 0x1f) - 1;
	tm->tm_year = bcd2bin((cal >> 16) & 0xff) + 100;
}

static void nuc970_rtc_tm2bcd(const struct rtc_time *tm, u32 *time, u32 *cal)
{
	*time = bin2bcd(tm->tm_sec) | bin2bcd(tm->tm_min) << 8 |
		bin2bcd(tm->tm_hour) << 16;
	*cal = bin2bcd(tm->tm_mday) | bin2bcd(tm->tm_mon + 1) << 8 |
	       bin2bcd(tm->tm_year - 100) << 16;
}

static irqreturn_t nuc970_rtc_irq(int irq, void *dev_id)
{
	struct nuc970_rtc *p = dev_id;
	u32 st;

	st = readl(p->base + RTC_RIIR) & (RTC_INT_ALARM | RTC_INT_TICK);
	if (!st)
		return IRQ_NONE;

	if (st & RTC_INT_ALARM) {
		nuc970_rtc_write(p, RTC_RIIR, RTC_INT_ALARM);
		rtc_update_irq(p->rtc, 1, RTC_AF | RTC_IRQF);
	}

	if (st & RTC_INT_TICK)
		nuc970_rtc_write(p, RTC_RIIR, RTC_INT_TICK);

	return IRQ_HANDLED;
}

static int nuc970_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);
	u32 time, cal;

	/* re-read if the calendar rolled over between the two reads */
	do {
		cal = readl(p->base + RTC_CLR);
		time = readl(p->base + RTC_TLR);
	} while (cal != readl(p->base + RTC_CLR));

	nuc970_rtc_bcd2tm(time, cal, tm);

	return 0;
}

static int nuc970_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);
	u32 time, cal;
	int ret;

	nuc970_rtc_tm2bcd(tm, &time, &cal);

	ret = nuc970_rtc_access_enable(p);
	if (ret)
		return ret;

	nuc970_rtc_write(p, RTC_CLR, cal);
	nuc970_rtc_write(p, RTC_TLR, time);

	return 0;
}

static int nuc970_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);

	if (enabled)
		nuc970_rtc_rmw(p, RTC_RIER, 0, RTC_INT_ALARM);
	else
		nuc970_rtc_rmw(p, RTC_RIER, RTC_INT_ALARM, 0);

	return 0;
}

static int nuc970_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);
	int ret;

	/* the BSP unlocks before reading the alarm registers */
	ret = nuc970_rtc_access_enable(p);
	if (ret)
		return ret;

	nuc970_rtc_bcd2tm(readl(p->base + RTC_TAR), readl(p->base + RTC_CAR),
			  &alrm->time);
	alrm->enabled = !!(readl(p->base + RTC_RIER) & RTC_INT_ALARM);
	alrm->pending = !!(readl(p->base + RTC_RIIR) & RTC_INT_ALARM);

	return 0;
}

static int nuc970_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);
	u32 time, cal;
	int ret;

	nuc970_rtc_tm2bcd(&alrm->time, &time, &cal);

	ret = nuc970_rtc_access_enable(p);
	if (ret)
		return ret;

	nuc970_rtc_write(p, RTC_CAR, cal | RTC_CAR_NO_WDAY);
	nuc970_rtc_write(p, RTC_TAR, time);
	nuc970_rtc_rmw(p, RTC_PWRCTL, 0, RTC_PWRCTL_ALARM);

	/* the RTC core expects set_alarm to honour alrm->enabled */
	return nuc970_rtc_alarm_irq_enable(dev, alrm->enabled);
}

static int nuc970_rtc_ioctl(struct device *dev, unsigned int cmd,
			    unsigned long arg)
{
	struct nuc970_rtc *p = dev_get_drvdata(dev);
	void __user *uarg = (void __user *)arg;
	u32 spare[RTC_NUM_SPARE];
	unsigned int i;
	int ret;

	switch (cmd) {
	case NUC970_RTC_GET_SPARE_DATA:
		ret = nuc970_rtc_access_enable(p);
		if (ret)
			return ret;

		for (i = 0; i < RTC_NUM_SPARE; i++)
			spare[i] = readl(p->base + RTC_SPR(i));

		if (copy_to_user(uarg, spare, sizeof(spare)))
			return -EFAULT;
		return 0;

	case NUC970_RTC_SET_SPARE_DATA:
		if (copy_from_user(spare, uarg, sizeof(spare)))
			return -EFAULT;

		ret = nuc970_rtc_access_enable(p);
		if (ret)
			return ret;

		for (i = 0; i < RTC_NUM_SPARE; i++)
			nuc970_rtc_write(p, RTC_SPR(i), spare[i]);
		return 0;

	case NUC970_SYS_GET_RSTSTS:
		if (!p->gcr)
			return -ENODEV;
		return put_user(readl(p->gcr + GCR_RSTSTS), (u32 __user *)uarg);

	case NUC970_SYS_RESET_RSTSTS:
		if (!p->gcr)
			return -ENODEV;
		/* write 1 to clear: write back whatever is currently set */
		writel(readl(p->gcr + GCR_RSTSTS), p->gcr + GCR_RSTSTS);
		return 0;

	default:
		return -ENOIOCTLCMD;
	}
}

static const struct rtc_class_ops nuc970_rtc_ops = {
	.read_time		= nuc970_rtc_read_time,
	.set_time		= nuc970_rtc_set_time,
	.read_alarm		= nuc970_rtc_read_alarm,
	.set_alarm		= nuc970_rtc_set_alarm,
	.alarm_irq_enable	= nuc970_rtc_alarm_irq_enable,
	.ioctl			= nuc970_rtc_ioctl,
};

static void nuc970_rtc_iounmap(void *addr)
{
	iounmap(addr);
}

/* The GCR is shared with other users; map it without claiming the region. */
static void __iomem *nuc970_rtc_map_gcr(struct device *dev)
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

	if (devm_add_action_or_reset(dev, nuc970_rtc_iounmap, (void __force *)gcr))
		return NULL;

	return gcr;
}

static int nuc970_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_rtc *p;
	struct clk *clk;
	int irq, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	p->gcr = nuc970_rtc_map_gcr(dev);
	if (!p->gcr)
		dev_warn(dev, "no GCR mapping, reset-status ioctls disabled\n");

	platform_set_drvdata(pdev, p);

	p->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(p->rtc))
		return PTR_ERR(p->rtc);

	p->rtc->ops = &nuc970_rtc_ops;
	p->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	p->rtc->range_max = RTC_TIMESTAMP_END_2099;

	/* 24-hour mode, as the BSP sets it at probe (no unlock) */
	nuc970_rtc_rmw(p, RTC_TSSR, 0, RTC_TSSR_24H);

	/* the core emulates update interrupts; keep the hardware tick off */
	nuc970_rtc_rmw(p, RTC_RIER, RTC_INT_TICK, 0);

	ret = devm_request_irq(dev, irq, nuc970_rtc_irq, 0, dev_name(dev), p);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq %d\n", irq);

	if (p->gcr && of_property_read_bool(dev->of_node, "wakeup-source")) {
		writel(readl(p->gcr + GCR_WKUPSER) | GCR_WKUPSER_RTC,
		       p->gcr + GCR_WKUPSER);
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
		ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			return ret;
	}

	return devm_rtc_register_device(p->rtc);
}

static const struct of_device_id nuc970_rtc_of_match[] = {
	{ .compatible = "nuvoton,nuc970-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_rtc_of_match);

static struct platform_driver nuc970_rtc_driver = {
	.probe	= nuc970_rtc_probe,
	.driver	= {
		.name		= "nuc970-rtc",
		.of_match_table	= nuc970_rtc_of_match,
	},
};
module_platform_driver(nuc970_rtc_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 RTC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-rtc");
