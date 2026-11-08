// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Nuvoton NUC970/NUC977 enhanced timer (ETIMER) character device
 *
 * Forward-ported from drivers/misc/nuc970-etimer.c of Nuvoton's NUC970 Linux
 * BSPs (3.10 and 5.10; Copyright (c) 2014 Nuvoton technology corporation),
 * including the changes ATEN made in the PE4102 vendor kernel that the
 * vendor userland depends on (recovered from the 1.0.069 firmware kernel):
 *
 *  - ETMR_IOC_START_SW_WDT / ETMR_IOC_CLEAR_SW_WDT: a software watchdog used
 *    by monitord. Once started it must be cleared at least every ~14 s
 *    (40 ticks of 350 ms), otherwise the board is reset.
 *  - ETIMER1 in periodic mode keeps running after close and stays busy
 *    (the vendor used its interrupt to feed the external watchdog; here the
 *    external watchdog is fed by arch/arm/mach-nuc970, so only the visible
 *    behaviour is kept).
 *  - capture modes use the "cap" pinctrl state (ETIMER0 capture on PB3 on
 *    the PE4102); toggle output fails with -EPERM unless a "tgl" state exists.
 *
 * Each channel is a misc device /dev/etimerN. The ioctl numbers and their
 * argument (one u32 read from user memory) match the BSP's
 * <mach/nuc970-etimer.h>, which the vendor userland was compiled against.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/*
 * userland ABI, identical to the BSP's <mach/nuc970-etimer.h>. The BSP
 * header encodes the size of an "unsigned int *" (4 bytes on 32-bit ARM);
 * spell it as a 4-byte type so the numbers are the same on every
 * architecture (COMPILE_TEST on 64-bit hosts).
 */
#define ETMR_IOC_MAGIC			'e'
#define ETMR_IOC_STOP			_IO(ETMR_IOC_MAGIC, 0)
#define ETMR_IOC_TOGGLE			_IOW(ETMR_IOC_MAGIC, 1, __u32)
#define ETMR_IOC_FREE_COUNTING		_IOW(ETMR_IOC_MAGIC, 2, __u32)
#define ETMR_IOC_TRIGGER_COUNTING	_IOW(ETMR_IOC_MAGIC, 3, __u32)
#define ETMR_IOC_PERIODIC		_IOW(ETMR_IOC_MAGIC, 4, __u32)
#define ETMR_IOC_PERIODIC_FOR_WKUP	_IOW(ETMR_IOC_MAGIC, 5, __u32)
#define ETMR_IOC_CLKLXT			_IOW(ETMR_IOC_MAGIC, 6, __u32)
#define ETMR_IOC_CLKHXT			_IOW(ETMR_IOC_MAGIC, 7, __u32)
/* ATEN vendor kernel additions (argument ignored) */
#define ETMR_IOC_START_SW_WDT		_IOW(ETMR_IOC_MAGIC, 8, __u32)
#define ETMR_IOC_CLEAR_SW_WDT		_IOW(ETMR_IOC_MAGIC, 9, __u32)

/* the numbers the vendor binaries pass to ioctl() */
static_assert(ETMR_IOC_STOP == 0x00006500);
static_assert(ETMR_IOC_TOGGLE == 0x40046501);
static_assert(ETMR_IOC_FREE_COUNTING == 0x40046502);
static_assert(ETMR_IOC_TRIGGER_COUNTING == 0x40046503);
static_assert(ETMR_IOC_PERIODIC == 0x40046504);
static_assert(ETMR_IOC_PERIODIC_FOR_WKUP == 0x40046505);
static_assert(ETMR_IOC_CLKLXT == 0x40046506);
static_assert(ETMR_IOC_CLKHXT == 0x40046507);
static_assert(ETMR_IOC_START_SW_WDT == 0x40046508);
static_assert(ETMR_IOC_CLEAR_SW_WDT == 0x40046509);

/* registers, per channel */
#define ETMR_CTL		0x00
#define ETMR_PRECNT		0x04
#define ETMR_CMPR		0x08
#define ETMR_IER		0x0c
#define ETMR_ISR		0x10
#define ETMR_DR			0x14
#define ETMR_TCAP		0x18

#define ETMR_CTL_ETMR_EN	BIT(0)
#define ETMR_CTL_TWAKE_EN	BIT(2)
#define ETMR_CTL_PERIODIC	(1 << 4)
#define ETMR_CTL_TOGGLE		(2 << 4)
#define ETMR_CTL_TCAP_EN	BIT(16)
#define ETMR_CTL_TRIGGER_CNT	BIT(20)

#define ETMR_IER_TIE		BIT(0)
#define ETMR_IER_TCAP_IE	BIT(1)

#define ETMR_ISR_TIF		BIT(0)
#define ETMR_ISR_TCAPIF		BIT(1)
#define ETMR_ISR_TWKF		BIT(4)

#define ETMR_CMPR_MAX		0xffffff	/* 24-bit compare register */
#define ETMR_PRESCALE_1MHZ	11		/* 12 MHz / (11 + 1) */

#define ETMR_FREE_COUNTING	(ETMR_CTL_TCAP_EN | ETMR_CTL_PERIODIC | \
				 ETMR_CTL_ETMR_EN)
#define ETMR_TRIGGER_COUNTING	(ETMR_CTL_TRIGGER_CNT | ETMR_FREE_COUNTING)

enum nuc970_etimer_mode {
	ETIMER_MODE_NONE = 0,
	ETIMER_MODE_PERIODIC = 2,
	ETIMER_MODE_TOGGLE = 4,
	ETIMER_MODE_TRIGGER_COUNTING = 5,
	ETIMER_MODE_FREE_COUNTING = 6,
};

#define ETIMER_NR_CHANNELS	4

/*
 * Vendor kernel: a periodic ETIMER1 is left running on close and the device
 * stays busy (bin/etimer starts it once from rcS and exits).
 */
#define ETIMER_PERSISTENT_CH	1

/*
 * Vendor software watchdog: the vendor kernel stopped feeding the external
 * watchdog once 40 keep-alive ticks (350 ms each) passed without
 * ETMR_IOC_CLEAR_SW_WDT; the external watchdog IC then reset the board.
 * The external watchdog is fed elsewhere in this kernel, so emulate it: after
 * the 40 ticks plus the time the watchdog IC would need to fire, restart.
 */
#define SW_WDT_TICK_MS		350
#define SW_WDT_LIMIT		40
#define SW_WDT_GRACE		5

struct nuc970_etimer {
	struct device *dev;
	void __iomem *base;
	struct clk *eclk;
	struct clk *eclk_mux;
	struct clk *hxt;
	struct clk *lxt;
	struct pinctrl *pinctrl;
	struct miscdevice miscdev;
	char name[12];
	wait_queue_head_t wq;
	spinlock_t lock;		/* registers and the fields below */
	u32 cap;			/* latest capture result */
	u32 cnt;			/* time-out count returned by read() */
	u32 tick;			/* time-out counter */
	u32 t0;				/* free counting: first capture */
	u8 cap_state;			/* free counting: capture sequence */
	u8 ch;
	u8 mode;
	bool update;			/* new data for read()/poll() */
	bool occupied;
};

static struct {
	spinlock_t lock;		/* protects the fields below */
	bool enabled;
	unsigned int count;
} nuc970_sw_wdt = {
	.lock = __SPIN_LOCK_UNLOCKED(nuc970_sw_wdt.lock),
};

static void nuc970_sw_wdt_tick(struct timer_list *unused);
static DEFINE_TIMER(nuc970_sw_wdt_timer, nuc970_sw_wdt_tick);

static void nuc970_sw_wdt_tick(struct timer_list *unused)
{
	unsigned long flags;
	unsigned int count;

	spin_lock_irqsave(&nuc970_sw_wdt.lock, flags);
	count = ++nuc970_sw_wdt.count;
	spin_unlock_irqrestore(&nuc970_sw_wdt.lock, flags);

	if (count == SW_WDT_LIMIT)
		pr_emerg("nuc970-etimer: software watchdog not cleared for %u ms, resetting\n",
			 SW_WDT_LIMIT * SW_WDT_TICK_MS);

	if (count >= SW_WDT_LIMIT + SW_WDT_GRACE) {
		emergency_restart();
		return;
	}

	mod_timer(&nuc970_sw_wdt_timer,
		  jiffies + msecs_to_jiffies(SW_WDT_TICK_MS));
}

static void nuc970_sw_wdt_start(void)
{
	unsigned long flags;
	bool start;

	spin_lock_irqsave(&nuc970_sw_wdt.lock, flags);
	start = !nuc970_sw_wdt.enabled;
	nuc970_sw_wdt.enabled = true;
	spin_unlock_irqrestore(&nuc970_sw_wdt.lock, flags);

	/* like the vendor kernel, there is no way to stop it again */
	if (start)
		mod_timer(&nuc970_sw_wdt_timer,
			  jiffies + msecs_to_jiffies(SW_WDT_TICK_MS));
}

static void nuc970_sw_wdt_clear(void)
{
	unsigned long flags;

	spin_lock_irqsave(&nuc970_sw_wdt.lock, flags);
	nuc970_sw_wdt.count = 0;
	spin_unlock_irqrestore(&nuc970_sw_wdt.lock, flags);
}

static inline void etimer_writel(struct nuc970_etimer *t, u32 val,
				 unsigned int reg)
{
	writel(val, t->base + reg);
}

static inline u32 etimer_readl(struct nuc970_etimer *t, unsigned int reg)
{
	return readl(t->base + reg);
}

static void nuc970_etimer_stop(struct nuc970_etimer *t)
{
	unsigned long flags;

	spin_lock_irqsave(&t->lock, flags);
	etimer_writel(t, 0, ETMR_CTL);
	etimer_writel(t, 0, ETMR_IER);
	etimer_writel(t, 0xffffffff, ETMR_ISR);
	t->mode = ETIMER_MODE_NONE;
	t->update = false;
	spin_unlock_irqrestore(&t->lock, flags);
}

/* ETMR_IOC_CLKLXT / ETMR_IOC_CLKHXT: engine clock from 32.768 kHz / 12 MHz */
static void nuc970_etimer_set_clksrc(struct nuc970_etimer *t, bool lxt)
{
	struct clk *parent = lxt ? t->lxt : t->hxt;
	int ret;

	etimer_writel(t, lxt ? 0 : ETMR_PRESCALE_1MHZ, ETMR_PRECNT);
	if (!parent || !t->eclk_mux)
		return;

	ret = clk_set_parent(t->eclk_mux, parent);
	if (ret)
		dev_dbg(t->dev, "cannot switch engine clock: %d\n", ret);
}

static bool nuc970_etimer_on_lxt(struct nuc970_etimer *t)
{
	if (t->lxt && t->eclk_mux)
		return clk_is_match(clk_get_parent(t->eclk_mux), t->lxt);

	return clk_get_rate(t->eclk) == 32768;
}

static int nuc970_etimer_select_pins(struct nuc970_etimer *t,
				     const char *state)
{
	struct pinctrl_state *s;

	if (IS_ERR_OR_NULL(t->pinctrl))
		return -EPERM;

	s = pinctrl_lookup_state(t->pinctrl, state);
	if (IS_ERR(s))
		return -EPERM;

	return pinctrl_select_state(t->pinctrl, s);
}

static irqreturn_t nuc970_etimer_interrupt(int irq, void *dev_id)
{
	struct nuc970_etimer *t = dev_id;
	u32 isr, tcap;

	spin_lock(&t->lock);

	isr = etimer_readl(t, ETMR_ISR);
	if (!(isr & (ETMR_ISR_TIF | ETMR_ISR_TCAPIF | ETMR_ISR_TWKF))) {
		spin_unlock(&t->lock);
		return IRQ_NONE;
	}

	if (isr & ETMR_ISR_TWKF)
		etimer_writel(t, ETMR_ISR_TWKF, ETMR_ISR);

	if (isr & ETMR_ISR_TIF) {
		t->cnt = t->tick++;
		etimer_writel(t, ETMR_ISR_TIF, ETMR_ISR);
		t->update = true;
	}

	if (isr & ETMR_ISR_TCAPIF) {
		tcap = etimer_readl(t, ETMR_TCAP);

		if (t->mode == ETIMER_MODE_FREE_COUNTING) {
			/*
			 * Measure the input frequency (Hz, 1 MHz counter) from
			 * two consecutive captures, then skip one.
			 */
			switch (t->cap_state) {
			case 0:
				t->t0 = tcap;
				t->cap_state = 1;
				break;
			case 1:
				t->cap_state = 2;
				/* t0 > tcap: counter wrapped, drop it */
				if (tcap > t->t0) {
					t->cap = 1000000 / (tcap - t->t0);
					t->update = true;
				}
				break;
			default:
				t->cap_state = 0;
				break;
			}
		} else {
			t->cap = tcap;
			t->update = true;
		}

		etimer_writel(t, ETMR_ISR_TCAPIF, ETMR_ISR);
	}

	spin_unlock(&t->lock);

	wake_up_interruptible(&t->wq);

	return IRQ_HANDLED;
}

static struct nuc970_etimer *file_to_etimer(struct file *filp)
{
	struct miscdevice *misc = filp->private_data;

	return container_of(misc, struct nuc970_etimer, miscdev);
}

static int nuc970_etimer_open(struct inode *inode, struct file *filp)
{
	struct nuc970_etimer *t = file_to_etimer(filp);
	unsigned long flags;

	spin_lock_irqsave(&t->lock, flags);
	if (t->occupied) {
		spin_unlock_irqrestore(&t->lock, flags);
		return -EBUSY;
	}
	t->occupied = true;
	spin_unlock_irqrestore(&t->lock, flags);

	/* every open starts from the 12 MHz crystal, as in the BSP */
	if (t->hxt && t->eclk_mux && clk_set_parent(t->eclk_mux, t->hxt))
		dev_dbg(t->dev, "cannot select 12 MHz engine clock\n");

	return 0;
}

static int nuc970_etimer_release(struct inode *inode, struct file *filp)
{
	struct nuc970_etimer *t = file_to_etimer(filp);
	unsigned long flags;

	/* vendor kernel: periodic ETIMER1 keeps running and stays busy */
	if (t->ch == ETIMER_PERSISTENT_CH &&
	    READ_ONCE(t->mode) == ETIMER_MODE_PERIODIC)
		return 0;

	nuc970_etimer_stop(t);

	spin_lock_irqsave(&t->lock, flags);
	t->occupied = false;
	spin_unlock_irqrestore(&t->lock, flags);

	return 0;
}

static ssize_t nuc970_etimer_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct nuc970_etimer *t = file_to_etimer(filp);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&t->lock, flags);

	if (t->mode != ETIMER_MODE_TRIGGER_COUNTING &&
	    t->mode != ETIMER_MODE_FREE_COUNTING &&
	    t->mode != ETIMER_MODE_PERIODIC) {
		spin_unlock_irqrestore(&t->lock, flags);
		return -EPERM;
	}

	if (!t->update) {
		spin_unlock_irqrestore(&t->lock, flags);
		/*
		 * Like the BSP, a signal ends the wait but the (stale) value
		 * is still returned: the vendor userland uses read() as a
		 * blocking delay and does not expect -EINTR.
		 */
		if (wait_event_interruptible(t->wq, READ_ONCE(t->update)))
			dev_dbg(t->dev, "read interrupted\n");
		spin_lock_irqsave(&t->lock, flags);
	}

	switch (t->mode) {
	case ETIMER_MODE_TRIGGER_COUNTING:
	case ETIMER_MODE_FREE_COUNTING:
		val = t->cap;
		break;
	case ETIMER_MODE_PERIODIC:
		val = t->cnt;
		break;
	default:
		/* stopped while waiting */
		t->update = false;
		spin_unlock_irqrestore(&t->lock, flags);
		return 0;
	}
	t->update = false;

	spin_unlock_irqrestore(&t->lock, flags);

	/* the BSP always copies one unsigned int */
	if (copy_to_user(buf, &val, sizeof(val)))
		return -EFAULT;

	return sizeof(val);
}

static __poll_t nuc970_etimer_poll(struct file *filp, poll_table *wait)
{
	struct nuc970_etimer *t = file_to_etimer(filp);

	poll_wait(filp, &t->wq, wait);

	return READ_ONCE(t->update) ? EPOLLIN | EPOLLRDNORM : 0;
}

static void nuc970_etimer_start(struct nuc970_etimer *t, u32 prescale,
				bool set_prescale, u32 cmpr, u32 ier, u32 ctl,
				u8 mode)
{
	unsigned long flags;

	spin_lock_irqsave(&t->lock, flags);
	if (set_prescale)
		etimer_writel(t, prescale, ETMR_PRECNT);
	etimer_writel(t, cmpr, ETMR_CMPR);
	if (ier)
		etimer_writel(t, ier, ETMR_IER);
	etimer_writel(t, ctl, ETMR_CTL);
	t->mode = mode;
	spin_unlock_irqrestore(&t->lock, flags);
}

static long nuc970_etimer_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct nuc970_etimer *t = file_to_etimer(filp);
	u32 __user *argp = (u32 __user *)arg;
	unsigned long flags;
	u32 param;
	int ret;

	/* the BSP stops the channel before any command, even unknown ones */
	nuc970_etimer_stop(t);

	spin_lock_irqsave(&t->lock, flags);
	t->tick = 1;
	t->cnt = 1;
	t->cap_state = 0;
	spin_unlock_irqrestore(&t->lock, flags);

	switch (cmd) {
	case ETMR_IOC_STOP:
		return 0;

	case ETMR_IOC_CLKLXT:
		nuc970_etimer_set_clksrc(t, true);
		return 0;

	case ETMR_IOC_CLKHXT:
		nuc970_etimer_set_clksrc(t, false);
		return 0;

	case ETMR_IOC_START_SW_WDT:
		nuc970_sw_wdt_start();
		return 0;

	case ETMR_IOC_CLEAR_SW_WDT:
		nuc970_sw_wdt_clear();
		return 0;

	case ETMR_IOC_PERIODIC:
		/* param: period in us */
		if (get_user(param, argp))
			return -EFAULT;
		if (param > ETMR_CMPR_MAX)
			return -EPERM;
		nuc970_etimer_start(t, ETMR_PRESCALE_1MHZ, true, param,
				    ETMR_IER_TIE,
				    ETMR_CTL_PERIODIC | ETMR_CTL_ETMR_EN,
				    ETIMER_MODE_PERIODIC);
		return 0;

	case ETMR_IOC_PERIODIC_FOR_WKUP:
		if (get_user(param, argp))
			return -EFAULT;
		if (param > ETMR_CMPR_MAX)
			return -EPERM;
		/* power-down wake-up needs the 32.768 kHz engine clock */
		if (!nuc970_etimer_on_lxt(t)) {
			dev_info(t->dev,
				 "Power down mode clock need to switch to 32k.\n");
			return -EPERM;
		}
		/* the prescaler set by ETMR_IOC_CLKLXT is kept */
		nuc970_etimer_start(t, 0, false, param, ETMR_IER_TIE,
				    ETMR_CTL_PERIODIC | ETMR_CTL_ETMR_EN,
				    ETIMER_MODE_PERIODIC);
		return 0;

	case ETMR_IOC_TOGGLE:
		/* param: output period in us */
		if (get_user(param, argp))
			return -EFAULT;
		/* half a period high, half low */
		param >>= 1;
		if (param > ETMR_CMPR_MAX)
			return -EPERM;
		/* no toggle pin (all of them on the PE4102): -EPERM */
		ret = nuc970_etimer_select_pins(t, "tgl");
		if (ret)
			return ret;
		nuc970_etimer_start(t, ETMR_PRESCALE_1MHZ, true, param, 0,
				    ETMR_CTL_TOGGLE | ETMR_CTL_ETMR_EN,
				    ETIMER_MODE_TOGGLE);
		return 0;

	case ETMR_IOC_FREE_COUNTING:
	case ETMR_IOC_TRIGGER_COUNTING:
		/* param: capture edge (ETMR_CAP_EDGE_*), OR-ed into CTL */
		if (get_user(param, argp))
			return -EFAULT;
		ret = nuc970_etimer_select_pins(t, "cap");
		if (ret)
			return ret;
		if (cmd == ETMR_IOC_FREE_COUNTING)
			nuc970_etimer_start(t, ETMR_PRESCALE_1MHZ, true,
					    ETMR_CMPR_MAX, ETMR_IER_TCAP_IE,
					    param | ETMR_FREE_COUNTING,
					    ETIMER_MODE_FREE_COUNTING);
		else
			nuc970_etimer_start(t, ETMR_PRESCALE_1MHZ, true,
					    ETMR_CMPR_MAX, ETMR_IER_TCAP_IE,
					    param | ETMR_TRIGGER_COUNTING,
					    ETIMER_MODE_TRIGGER_COUNTING);
		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations nuc970_etimer_fops = {
	.owner		= THIS_MODULE,
	.open		= nuc970_etimer_open,
	.release	= nuc970_etimer_release,
	.read		= nuc970_etimer_read,
	.poll		= nuc970_etimer_poll,
	.unlocked_ioctl	= nuc970_etimer_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.llseek		= noop_llseek,
};

static int nuc970_etimer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_etimer *t;
	struct resource *res;
	struct clk *pclk;
	int irq, ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->dev = dev;
	spin_lock_init(&t->lock);
	init_waitqueue_head(&t->wq);

	t->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(t->base))
		return PTR_ERR(t->base);

	/* the four channels are 0x100 apart at 0xb8001400..0xb8001700 */
	t->ch = (res->start >> 8) & (ETIMER_NR_CHANNELS - 1);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/*
	 * The APB gate and the engine clock gate share the same enable bit:
	 * hold both, or clk_disable_unused() turns the timer off.
	 */
	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk), "no pclk\n");

	t->eclk = devm_clk_get_enabled(dev, "eclk");
	if (IS_ERR(t->eclk))
		return dev_err_probe(dev, PTR_ERR(t->eclk), "no eclk\n");

	/* engine clock source mux (12 MHz / 32.768 kHz) */
	t->eclk_mux = clk_get_parent(t->eclk);

	t->hxt = devm_clk_get_optional(dev, "hxt");
	if (IS_ERR(t->hxt))
		return dev_err_probe(dev, PTR_ERR(t->hxt), "bad hxt clock\n");

	t->lxt = devm_clk_get_optional(dev, "lxt");
	if (IS_ERR(t->lxt))
		return dev_err_probe(dev, PTR_ERR(t->lxt), "bad lxt clock\n");

	if (t->hxt && t->eclk_mux) {
		ret = clk_set_parent(t->eclk_mux, t->hxt);
		if (ret)
			dev_warn(dev, "cannot select 12 MHz engine clock: %d\n",
				 ret);
	}

	/* optional "cap"/"tgl" states, selected when a mode needs them */
	t->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(t->pinctrl)) {
		if (PTR_ERR(t->pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		t->pinctrl = NULL;
	}

	/* stop whatever the boot loader left running */
	etimer_writel(t, 0, ETMR_CTL);
	etimer_writel(t, 0, ETMR_IER);
	etimer_writel(t, 0xffffffff, ETMR_ISR);

	ret = devm_request_irq(dev, irq, nuc970_etimer_interrupt, 0,
			       dev_name(dev), t);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq %d\n", irq);

	snprintf(t->name, sizeof(t->name), "etimer%u", t->ch);
	t->miscdev.minor = MISC_DYNAMIC_MINOR;
	t->miscdev.name = t->name;
	t->miscdev.fops = &nuc970_etimer_fops;
	t->miscdev.parent = dev;

	platform_set_drvdata(pdev, t);

	ret = misc_register(&t->miscdev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register /dev/%s\n",
				     t->name);

	dev_dbg(dev, "/dev/%s, irq %d\n", t->name, irq);

	return 0;
}

static void nuc970_etimer_remove(struct platform_device *pdev)
{
	struct nuc970_etimer *t = platform_get_drvdata(pdev);

	misc_deregister(&t->miscdev);
	nuc970_etimer_stop(t);
}

static const struct of_device_id nuc970_etimer_of_match[] = {
	{ .compatible = "nuvoton,nuc970-etimer" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_etimer_of_match);

static struct platform_driver nuc970_etimer_driver = {
	.driver = {
		.name = "nuc970-etimer",
		.of_match_table = nuc970_etimer_of_match,
		/*
		 * An open /dev/etimerN (or the ETIMER1 that stays running
		 * after close) would outlive the devm-allocated state on a
		 * sysfs unbind.
		 */
		.suppress_bind_attrs = true,
	},
	.probe = nuc970_etimer_probe,
	.remove = nuc970_etimer_remove,
};

static int __init nuc970_etimer_init(void)
{
	return platform_driver_register(&nuc970_etimer_driver);
}
module_init(nuc970_etimer_init);

static void __exit nuc970_etimer_exit(void)
{
	platform_driver_unregister(&nuc970_etimer_driver);
	timer_delete_sync(&nuc970_sw_wdt_timer);
}
module_exit(nuc970_etimer_exit);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("Nuvoton NUC970 ETIMER character device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-etimer");
