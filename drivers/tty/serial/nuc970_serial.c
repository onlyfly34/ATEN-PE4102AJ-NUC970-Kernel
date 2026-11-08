// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Nuvoton NUC970/NUC977 UART driver
 *
 * Forward-ported from drivers/tty/serial/nuc970_serial.c in Nuvoton's NUC970
 * Linux BSP (Copyright (c) 2014 Nuvoton technology corporation), rewritten
 * for the current serial core: DT probing, per-port clocks, kfifo TX via
 * uart_port_tx(), console and earlycon.
 *
 * RS485 follows the BSP's default build (DISABLE_AUTO_DIRECTION undefined):
 * the UART is switched to its RS485 function with hardware auto-direction
 * control, so the controller itself drives RTS for the whole frame and
 * releases it after the last stop bit. TIOCSRS485/TIOCGRS485 and the DT
 * rs485 properties are handled by the serial core.
 *
 * Ports are named ttyS<n> where <n> comes from the "serialN" DT alias, so the
 * vendor userland keeps its ttyS0 (console) / ttyS6 (metering MCU) names.
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define UART_NR			11
#define NUC970_UART_FIFO	16

/* registers */
#define REG_RBR		0x00
#define REG_THR		0x00
#define REG_IER		0x04
#define  IER_RDA	BIT(0)
#define  IER_THRE	BIT(1)
#define  IER_RTO	BIT(4)
#define  IER_TIME_OUT_EN BIT(11)
#define  IER_AUTO_RTS	BIT(12)
#define  IER_AUTO_CTS	BIT(13)
#define REG_FCR		0x08
#define  FCR_RFR	BIT(1)
#define  FCR_TFR	BIT(2)
#define  FCR_RFITL_4	(0x1 << 4)
#define  FCR_RTS_TRI_8	(0x2 << 16)
#define REG_LCR		0x0c
#define  LCR_WLS_MASK	0x3
#define  LCR_NSB	BIT(2)
#define  LCR_PBE	BIT(3)
#define  LCR_EPE	BIT(4)
#define  LCR_SPE	BIT(5)
#define  LCR_BCB	BIT(6)
#define REG_MCR		0x10
#define  MCR_RTS	BIT(1)
#define  MCR_LEV_RTS	BIT(9)
#define REG_MSR		0x14
#define  MSR_CTS_ST	BIT(4)
#define  MSR_LEV_CTS	BIT(8)
#define REG_FSR		0x18
#define  FSR_RX_OVER	BIT(0)
#define  FSR_PEF	BIT(4)
#define  FSR_FEF	BIT(5)
#define  FSR_BIF	BIT(6)
#define  FSR_RX_EMPTY	BIT(14)
#define  FSR_TX_EMPTY	BIT(22)
#define  FSR_TX_FULL	BIT(23)
#define  FSR_TE		BIT(28)
#define  FSR_ERR	(FSR_RX_OVER | FSR_PEF | FSR_FEF | FSR_BIF)
#define REG_ISR		0x1c
#define  ISR_RDA	BIT(0)
#define  ISR_THRE	BIT(1)
#define  ISR_TOUT	BIT(4)
#define  ISR_THRE_INT	BIT(9)
#define REG_TOR		0x20
#define REG_BAUD	0x24
#define  BAUD_MODE2	(0x3 << 28)	/* DIV_X_EN | DIV_X_ONE: clk/(BRD+2) */
#define  BAUD_BRD_MASK	0xffff
#define REG_ALT_CSR	0x2c
#define  ALT_CSR_RS485_NMM BIT(8)	/* normal multidrop mode */
#define  ALT_CSR_RS485_AAD BIT(9)	/* auto address detection */
#define  ALT_CSR_RS485_AUD BIT(10)	/* auto direction control */
#define REG_FUN_SEL	0x30
#define  FUN_SEL_MASK	0x7
#define  FUN_SEL_UART	0x0
#define  FUN_SEL_RS485	0x3

struct nuc970_uart {
	struct uart_port port;
	struct clk *pclk;
	struct clk *eclk;
};

static struct nuc970_uart nuc970_uart_ports[UART_NR];

static inline struct nuc970_uart *to_nuc970_uart(struct uart_port *port)
{
	return container_of(port, struct nuc970_uart, port);
}

static inline u32 nuc970_read(struct uart_port *port, unsigned int off)
{
	return readl_relaxed(port->membase + off);
}

static inline void nuc970_write(struct uart_port *port, unsigned int off,
				u32 val)
{
	writel_relaxed(val, port->membase + off);
}

static unsigned int nuc970_tx_empty(struct uart_port *port)
{
	u32 fsr = nuc970_read(port, REG_FSR);

	return (fsr & (FSR_TE | FSR_TX_EMPTY)) == (FSR_TE | FSR_TX_EMPTY) ?
		TIOCSER_TEMT : 0;
}

static unsigned int nuc970_get_mctrl(struct uart_port *port)
{
	unsigned int ret = TIOCM_DSR | TIOCM_CAR;

	if (!(nuc970_read(port, REG_MSR) & MSR_CTS_ST))
		ret |= TIOCM_CTS;
	return ret;
}

/*
 * MCR value for RS485 auto-direction mode: the hardware asserts RTS while
 * transmitting and LEV_RTS selects the pin level of "asserted" (clear: high,
 * as in the BSP's SER_RS485_RTS_ON_SEND path). The software RTS bit is left
 * inactive so the pin already sits at the receive level before the function
 * switch.
 */
static u32 nuc970_rs485_mcr(u32 mcr, const struct serial_rs485 *rs485)
{
	mcr |= MCR_RTS;
	if (rs485->flags & SER_RS485_RTS_ON_SEND)
		mcr &= ~MCR_LEV_RTS;
	else
		mcr |= MCR_LEV_RTS;
	return mcr;
}

static void nuc970_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 mcr = nuc970_read(port, REG_MCR);
	u32 ier = nuc970_read(port, REG_IER);

	ier &= ~(IER_AUTO_RTS | IER_AUTO_CTS);

	if (port->rs485.flags & SER_RS485_ENABLED) {
		/* RTS belongs to the auto-direction logic; no flow control */
		mcr = nuc970_rs485_mcr(mcr, &port->rs485);
	} else {
		/* active-low signals: LEV_RTS selects high-level trigger */
		mcr |= MCR_LEV_RTS;
		if (mctrl & TIOCM_RTS)
			mcr &= ~MCR_RTS;
		else
			mcr |= MCR_RTS;

		/*
		 * Hardware RTS flow control follows TIOCM_RTS: the core
		 * throttles by clearing it (no UPSTAT_AUTORTS, so it never
		 * calls the absent ->throttle op), which drops back to the
		 * inactive software RTS above.
		 */
		if (port->status & UPSTAT_AUTOCTS) {
			ier |= IER_AUTO_CTS;
			if (mctrl & TIOCM_RTS)
				ier |= IER_AUTO_RTS;
		}
	}

	nuc970_write(port, REG_MSR, nuc970_read(port, REG_MSR) | MSR_LEV_CTS);
	nuc970_write(port, REG_MCR, mcr);
	nuc970_write(port, REG_IER, ier);
}

static void nuc970_stop_tx(struct uart_port *port)
{
	nuc970_write(port, REG_IER, nuc970_read(port, REG_IER) & ~IER_THRE);
}

static void nuc970_tx_chars(struct uart_port *port)
{
	u8 ch;

	uart_port_tx(port, ch,
		     !(nuc970_read(port, REG_FSR) & FSR_TX_FULL),
		     nuc970_write(port, REG_THR, ch));
}

static void nuc970_start_tx(struct uart_port *port)
{
	u32 ier = nuc970_read(port, REG_IER);

	nuc970_write(port, REG_IER, ier & ~IER_THRE);
	nuc970_tx_chars(port);
	/* uart_port_tx() calls stop_tx when the kfifo drains */
	if (!kfifo_is_empty(&port->state->port.xmit_fifo) || port->x_char)
		nuc970_write(port, REG_IER, ier | IER_THRE);
}

static void nuc970_stop_rx(struct uart_port *port)
{
	nuc970_write(port, REG_IER,
		     nuc970_read(port, REG_IER) & ~(IER_RDA | IER_RTO));
}

static void nuc970_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	u32 lcr;

	uart_port_lock_irqsave(port, &flags);
	lcr = nuc970_read(port, REG_LCR);
	if (break_state)
		lcr |= LCR_BCB;
	else
		lcr &= ~LCR_BCB;
	nuc970_write(port, REG_LCR, lcr);
	uart_port_unlock_irqrestore(port, flags);
}

static void nuc970_rx_chars(struct uart_port *port)
{
	unsigned int max = 256;
	u32 fsr;
	u8 ch, flag;

	while (!((fsr = nuc970_read(port, REG_FSR)) & FSR_RX_EMPTY) && max--) {
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(fsr & FSR_ERR)) {
			nuc970_write(port, REG_FSR, fsr & FSR_ERR);
			if (fsr & FSR_BIF) {
				port->icount.brk++;
				if (uart_handle_break(port)) {
					nuc970_read(port, REG_RBR);
					continue;
				}
			}
			if (fsr & FSR_FEF)
				port->icount.frame++;
			if (fsr & FSR_PEF)
				port->icount.parity++;
			if (fsr & FSR_RX_OVER)
				port->icount.overrun++;

			fsr &= port->read_status_mask;
			if (fsr & FSR_BIF)
				flag = TTY_BREAK;
			else if (fsr & FSR_PEF)
				flag = TTY_PARITY;
			else if (fsr & FSR_FEF)
				flag = TTY_FRAME;
		}

		ch = nuc970_read(port, REG_RBR);
		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, fsr, FSR_RX_OVER, ch, flag);
	}

	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t nuc970_uart_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 isr = nuc970_read(port, REG_ISR);
	u32 ier = nuc970_read(port, REG_IER);

	if (!(isr & (ISR_RDA | ISR_TOUT | ISR_THRE_INT)))
		return IRQ_NONE;

	if (isr & (ISR_RDA | ISR_TOUT))
		nuc970_rx_chars(port);

	if ((isr & ISR_THRE_INT) && (ier & IER_THRE)) {
		uart_port_lock(port);
		nuc970_tx_chars(port);
		uart_port_unlock(port);
	}

	return IRQ_HANDLED;
}

static int nuc970_startup(struct uart_port *port)
{
	int ret;

	nuc970_write(port, REG_FCR, FCR_TFR | FCR_RFR);
	nuc970_write(port, REG_ISR, 0xffffffff);
	nuc970_write(port, REG_FSR, FSR_ERR);

	ret = request_irq(port->irq, nuc970_uart_irq, 0,
			  dev_name(port->dev), port);
	if (ret)
		return ret;

	nuc970_write(port, REG_FCR, nuc970_read(port, REG_FCR) |
		     FCR_RFITL_4 | FCR_RTS_TRI_8);
	nuc970_write(port, REG_TOR, 0x40);
	nuc970_write(port, REG_IER, IER_RTO | IER_RDA | IER_TIME_OUT_EN);

	return 0;
}

static void nuc970_shutdown(struct uart_port *port)
{
	nuc970_write(port, REG_IER, 0);
	free_irq(port->irq, port);
}

static void nuc970_set_termios(struct uart_port *port, struct ktermios *termios,
			       const struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, quot;
	u32 lcr;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = 0;
		break;
	case CS6:
		lcr = 1;
		break;
	case CS7:
		lcr = 2;
		break;
	default:
		termios->c_cflag = (termios->c_cflag & ~CSIZE) | CS8;
		lcr = 3;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= LCR_NSB;
	if (termios->c_cflag & PARENB) {
		lcr |= LCR_PBE;
		if (!(termios->c_cflag & PARODD))
			lcr |= LCR_EPE;
		if (termios->c_cflag & CMSPAR)
			lcr |= LCR_SPE;
	}

	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / (BAUD_BRD_MASK + 2),
				  port->uartclk / 11);
	quot = DIV_ROUND_CLOSEST(port->uartclk, baud) - 2;

	uart_port_lock_irqsave(port, &flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = FSR_RX_OVER;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= FSR_FEF | FSR_PEF;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= FSR_BIF;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= FSR_FEF | FSR_PEF;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= FSR_BIF;
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= FSR_RX_OVER;
	}

	/*
	 * Only UPSTAT_AUTOCTS: UPSTAT_AUTORTS would make uart_throttle() call
	 * ops->throttle, which this driver does not implement (NULL deref).
	 */
	if (termios->c_cflag & CRTSCTS)
		port->status |= UPSTAT_AUTOCTS;
	else
		port->status &= ~UPSTAT_AUTOCTS;
	nuc970_set_mctrl(port, port->mctrl);

	nuc970_write(port, REG_BAUD, BAUD_MODE2 | (quot & BAUD_BRD_MASK));
	nuc970_write(port, REG_LCR, lcr);

	uart_port_unlock_irqrestore(port, flags);
}

static const char *nuc970_type(struct uart_port *port)
{
	return "NUC970";
}

static void nuc970_release_port(struct uart_port *port)
{
}

static int nuc970_request_port(struct uart_port *port)
{
	return 0;
}

static void nuc970_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

static int nuc970_verify_port(struct uart_port *port,
			      struct serial_struct *ser)
{
	return -EINVAL;
}

/*
 * Called by the serial core with the port lock held, before port->rs485 is
 * updated, so only @rs485 describes the requested state. Mirrors the BSP's
 * nuc970serial_config_rs485() auto-direction path: FUN_SEL = RS485 plus
 * ALT_CSR.RS485_AUD, RTS polarity via MCR.LEV_RTS. The BSP had no hardware
 * RTS delays (it only stored the values), so none are advertised.
 */
static int nuc970_rs485_config(struct uart_port *port, struct ktermios *termios,
			       struct serial_rs485 *rs485)
{
	u32 fun = nuc970_read(port, REG_FUN_SEL) & ~FUN_SEL_MASK;
	u32 alt = nuc970_read(port, REG_ALT_CSR) &
		  ~(ALT_CSR_RS485_NMM | ALT_CSR_RS485_AAD | ALT_CSR_RS485_AUD);

	if (rs485->flags & SER_RS485_ENABLED) {
		nuc970_write(port, REG_IER, nuc970_read(port, REG_IER) &
			     ~(IER_AUTO_RTS | IER_AUTO_CTS));
		nuc970_write(port, REG_MCR,
			     nuc970_rs485_mcr(nuc970_read(port, REG_MCR), rs485));
		nuc970_write(port, REG_ALT_CSR, alt | ALT_CSR_RS485_AUD);
		nuc970_write(port, REG_FUN_SEL, fun | FUN_SEL_RS485);
	} else {
		/* the core restores RTS/flow control via set_mctrl() */
		nuc970_write(port, REG_FUN_SEL, fun | FUN_SEL_UART);
		nuc970_write(port, REG_ALT_CSR, alt);
	}

	return 0;
}

static const struct serial_rs485 nuc970_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND |
		 SER_RS485_RTS_AFTER_SEND,
};

#ifdef CONFIG_CONSOLE_POLL
static int nuc970_poll_get_char(struct uart_port *port)
{
	if (nuc970_read(port, REG_FSR) & FSR_RX_EMPTY)
		return NO_POLL_CHAR;
	return nuc970_read(port, REG_RBR) & 0xff;
}

static void nuc970_poll_put_char(struct uart_port *port, unsigned char c)
{
	while (nuc970_read(port, REG_FSR) & FSR_TX_FULL)
		cpu_relax();
	nuc970_write(port, REG_THR, c);
}
#endif

static const struct uart_ops nuc970_uart_ops = {
	.tx_empty	= nuc970_tx_empty,
	.set_mctrl	= nuc970_set_mctrl,
	.get_mctrl	= nuc970_get_mctrl,
	.stop_tx	= nuc970_stop_tx,
	.start_tx	= nuc970_start_tx,
	.stop_rx	= nuc970_stop_rx,
	.break_ctl	= nuc970_break_ctl,
	.startup	= nuc970_startup,
	.shutdown	= nuc970_shutdown,
	.set_termios	= nuc970_set_termios,
	.type		= nuc970_type,
	.release_port	= nuc970_release_port,
	.request_port	= nuc970_request_port,
	.config_port	= nuc970_config_port,
	.verify_port	= nuc970_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= nuc970_poll_get_char,
	.poll_put_char	= nuc970_poll_put_char,
#endif
};

#ifdef CONFIG_SERIAL_NUC970_CONSOLE
static void nuc970_console_putchar(struct uart_port *port, unsigned char ch)
{
	while (nuc970_read(port, REG_FSR) & FSR_TX_FULL)
		cpu_relax();
	nuc970_write(port, REG_THR, ch);
}

static void nuc970_console_write(struct console *co, const char *s,
				 unsigned int count)
{
	struct uart_port *port = &nuc970_uart_ports[co->index].port;
	unsigned long flags;
	int locked = 1;
	u32 ier;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else
		uart_port_lock_irqsave(port, &flags);

	ier = nuc970_read(port, REG_IER);
	nuc970_write(port, REG_IER, 0);

	uart_console_write(port, s, count, nuc970_console_putchar);

	while (!(nuc970_read(port, REG_FSR) & FSR_TX_EMPTY))
		cpu_relax();
	nuc970_write(port, REG_IER, ier);

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static int __init nuc970_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= UART_NR)
		co->index = 0;
	port = &nuc970_uart_ports[co->index].port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver nuc970_uart_driver;

static struct console nuc970_console = {
	.name		= "ttyS",
	.write		= nuc970_console_write,
	.device		= uart_console_device,
	.setup		= nuc970_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &nuc970_uart_driver,
};
#define NUC970_CONSOLE	(&nuc970_console)

static void nuc970_early_putchar(struct uart_port *port, unsigned char ch)
{
	while (readl_relaxed(port->membase + REG_FSR) & FSR_TX_FULL)
		cpu_relax();
	writel_relaxed(ch, port->membase + REG_THR);
}

static void nuc970_early_write(struct console *con, const char *s,
			       unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, nuc970_early_putchar);
}

static int __init nuc970_early_console_setup(struct earlycon_device *device,
					     const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;
	device->con->write = nuc970_early_write;
	return 0;
}
OF_EARLYCON_DECLARE(nuc970, "nuvoton,nuc970-uart", nuc970_early_console_setup);
#else
#define NUC970_CONSOLE	NULL
#endif

static struct uart_driver nuc970_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "nuc970_serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= UART_NR,
	.cons		= NUC970_CONSOLE,
};

static int nuc970_uart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_uart *up;
	struct resource *res;
	void __iomem *base;
	int line, irq, ret;

	line = of_alias_get_id(dev->of_node, "serial");
	if (line < 0 || line >= UART_NR) {
		dev_err(dev, "missing or invalid serial alias (%d)\n", line);
		return -EINVAL;
	}
	up = &nuc970_uart_ports[line];
	if (up->port.dev) {
		dev_err(dev, "ttyS%d already registered\n", line);
		return -EBUSY;
	}

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* the APB gate and the engine-clock gate share an enable bit */
	up->pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(up->pclk))
		return dev_err_probe(dev, PTR_ERR(up->pclk), "no pclk\n");
	up->eclk = devm_clk_get_enabled(dev, "eclk");
	if (IS_ERR(up->eclk))
		return dev_err_probe(dev, PTR_ERR(up->eclk), "no eclk\n");

	up->port.dev = dev;
	up->port.line = line;
	up->port.type = PORT_GENERIC;
	up->port.iotype = UPIO_MEM32;
	up->port.membase = base;
	up->port.mapbase = res->start;
	up->port.irq = irq;
	up->port.fifosize = NUC970_UART_FIFO;
	up->port.ops = &nuc970_uart_ops;
	up->port.rs485_config = nuc970_rs485_config;
	up->port.rs485_supported = nuc970_rs485_supported;
	up->port.flags = UPF_BOOT_AUTOCONF;
	up->port.has_sysrq = IS_ENABLED(CONFIG_SERIAL_NUC970_CONSOLE);
	up->port.uartclk = clk_get_rate(up->eclk);
	if (!up->port.uartclk) {
		dev_warn(dev, "engine clock rate unknown, assuming 12 MHz\n");
		up->port.uartclk = 12000000;
	}

	/* rs485-rts-active-low, rs485-rts-delay, linux,rs485-enabled-at-boot-time */
	ret = uart_get_rs485_mode(&up->port);
	if (ret)
		goto err_clear;

	/*
	 * Start from plain UART function unless RS485 is enabled at boot (the
	 * core then applies it from uart_add_one_port()).
	 */
	if (!(up->port.rs485.flags & SER_RS485_ENABLED))
		nuc970_rs485_config(&up->port, NULL, &up->port.rs485);

	platform_set_drvdata(pdev, up);

	ret = uart_add_one_port(&nuc970_uart_driver, &up->port);
	if (ret)
		goto err_clear;
	return 0;

err_clear:
	up->port.dev = NULL;
	return dev_err_probe(dev, ret, "failed to register ttyS%d\n", line);
}

static void nuc970_uart_remove(struct platform_device *pdev)
{
	struct nuc970_uart *up = platform_get_drvdata(pdev);

	uart_remove_one_port(&nuc970_uart_driver, &up->port);
	up->port.dev = NULL;
}

static const struct of_device_id nuc970_uart_of_match[] = {
	{ .compatible = "nuvoton,nuc970-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_uart_of_match);

static struct platform_driver nuc970_uart_platform_driver = {
	.probe	= nuc970_uart_probe,
	.remove	= nuc970_uart_remove,
	.driver	= {
		.name		= "nuc970-uart",
		.of_match_table	= nuc970_uart_of_match,
	},
};

static int __init nuc970_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&nuc970_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&nuc970_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&nuc970_uart_driver);
	return ret;
}

static void __exit nuc970_uart_exit(void)
{
	platform_driver_unregister(&nuc970_uart_platform_driver);
	uart_unregister_driver(&nuc970_uart_driver);
}

module_init(nuc970_uart_init);
module_exit(nuc970_uart_exit);

MODULE_DESCRIPTION("Nuvoton NUC970 UART driver");
MODULE_LICENSE("GPL");
