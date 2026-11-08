// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 SPI controller
 *
 * Rewritten from drivers/spi/spi-nuc970-p0.c in Nuvoton's NUC970 Linux BSP
 * (Copyright (c) 2014 Nuvoton technology corporation) as a polled
 * transfer_one() SPI host with dual/quad support, which is what spi-nor
 * needs to reach the SPI-NOR flash through the spi-mem fallback path.
 *
 * The controller has no FIFO: it shifts up to four words (TX0..TX3 /
 * RX0..RX3) of 8..32 bits per GO_BUSY kick. Byte streams are by default
 * packed into 32-bit big-endian words so that one kick moves 16 bytes; set
 * spi_nuc970.fast32=0 on the command line to fall back to the BSP's 8-bit
 * words (4 bytes per kick).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define REG_CNTRL		0x00
#define  CNTRL_GO_BUSY		BIT(0)
#define  CNTRL_RX_NEG		BIT(1)
#define  CNTRL_TX_NEG		BIT(2)
#define  CNTRL_TX_BIT_LEN	GENMASK(7, 3)	/* 0 means 32 bits */
#define  CNTRL_TX_NUM		GENMASK(9, 8)	/* words per kick - 1 */
#define  CNTRL_LSB		BIT(10)
#define  CNTRL_IF		BIT(16)		/* write 1 to clear */
#define  CNTRL_IE		BIT(17)
#define  CNTRL_DIR_OUT		BIT(20)		/* dual/quad direction */
#define  CNTRL_QUAD		BIT(21)
#define  CNTRL_DUAL		BIT(22)
#define  CNTRL_CLKP		BIT(31)
#define REG_DIVIDER		0x04
#define REG_SSR			0x08
#define  SSR_SS(cs)		BIT(cs)
#define  SSR_SS_LVL		BIT(2)
#define REG_RX(n)		(0x10 + 4 * (n))
#define REG_TX(n)		(0x10 + 4 * (n))

#define NUC970_SPI_WORDS	4
#define NUC970_SPI_NUM_CS	2
#define NUC970_SPI_TIMEOUT_US	20000

static bool fast32 = true;
module_param(fast32, bool, 0644);
MODULE_PARM_DESC(fast32, "pack byte streams into 32-bit words (default: on)");

struct nuc970_spi {
	void __iomem *base;
	struct clk *clk;
	u32 cur_speed;
};

static void nuc970_spi_set_cs(struct spi_device *spi, bool level)
{
	struct nuc970_spi *hw = spi_controller_get_devdata(spi->controller);
	unsigned int cs = spi_get_chipselect(spi, 0);
	bool cs_high = spi->mode & SPI_CS_HIGH;
	bool active = cs_high ? level : !level;
	u32 ssr = readl_relaxed(hw->base + REG_SSR);

	if (cs_high)
		ssr |= SSR_SS_LVL;
	else
		ssr &= ~SSR_SS_LVL;
	if (active)
		ssr |= SSR_SS(cs);
	else
		ssr &= ~SSR_SS(cs);
	writel_relaxed(ssr, hw->base + REG_SSR);
}

static void nuc970_spi_set_speed(struct nuc970_spi *hw, u32 hz)
{
	unsigned long rate = clk_get_rate(hw->clk);
	u32 div;

	if (hz == hw->cur_speed)
		return;

	div = DIV_ROUND_UP(rate, 2 * hz);
	div = div ? div - 1 : 0;
	writel_relaxed(min_t(u32, div, 0xffff), hw->base + REG_DIVIDER);
	hw->cur_speed = hz;
}

/* run one kick of @nwords words of @bits bits */
static int nuc970_spi_kick(struct nuc970_spi *hw, u32 cntrl,
			   unsigned int bits, unsigned int nwords,
			   const u32 *tx, u32 *rx)
{
	unsigned int i;
	u32 val;
	int ret;

	cntrl |= FIELD_PREP(CNTRL_TX_BIT_LEN, bits == 32 ? 0 : bits) |
		 FIELD_PREP(CNTRL_TX_NUM, nwords - 1);
	writel_relaxed(cntrl, hw->base + REG_CNTRL);
	for (i = 0; i < nwords; i++)
		writel_relaxed(tx ? tx[i] : 0xffffffff, hw->base + REG_TX(i));
	writel(cntrl | CNTRL_GO_BUSY, hw->base + REG_CNTRL);

	ret = readl_poll_timeout_atomic(hw->base + REG_CNTRL, val,
					!(val & CNTRL_GO_BUSY), 0,
					NUC970_SPI_TIMEOUT_US);
	writel_relaxed(cntrl | CNTRL_IF, hw->base + REG_CNTRL);
	if (ret)
		return ret;

	if (rx)
		for (i = 0; i < nwords; i++)
			rx[i] = readl_relaxed(hw->base + REG_RX(i));
	return 0;
}

static int nuc970_spi_transfer_one(struct spi_controller *host,
				   struct spi_device *spi,
				   struct spi_transfer *t)
{
	struct nuc970_spi *hw = spi_controller_get_devdata(host);
	unsigned int bpw = t->bits_per_word;
	unsigned int wbytes = bpw <= 8 ? 1 : bpw <= 16 ? 2 : 4;
	const u8 *tx = t->tx_buf;
	u8 *rx = t->rx_buf;
	unsigned int left = t->len;
	u32 txw[NUC970_SPI_WORDS], rxw[NUC970_SPI_WORDS];
	u32 cntrl = 0;
	int ret;

	nuc970_spi_set_speed(hw, t->speed_hz);

	if (spi->mode & SPI_CPOL)
		cntrl |= CNTRL_CLKP;
	/* BSP mapping: modes 0/2 drive on the falling edge */
	if (spi->mode & SPI_CPHA)
		cntrl |= CNTRL_RX_NEG;
	else
		cntrl |= CNTRL_TX_NEG;
	if (spi->mode & SPI_LSB_FIRST)
		cntrl |= CNTRL_LSB;

	if (tx && t->tx_nbits == SPI_NBITS_QUAD)
		cntrl |= CNTRL_QUAD | CNTRL_DIR_OUT;
	else if (tx && t->tx_nbits == SPI_NBITS_DUAL)
		cntrl |= CNTRL_DUAL | CNTRL_DIR_OUT;
	else if (rx && t->rx_nbits == SPI_NBITS_QUAD)
		cntrl |= CNTRL_QUAD;
	else if (rx && t->rx_nbits == SPI_NBITS_DUAL)
		cntrl |= CNTRL_DUAL;

	/* fast path: byte stream as big-endian 32-bit words, 16 bytes/kick */
	if (fast32 && bpw == 8 && !(spi->mode & SPI_LSB_FIRST)) {
		while (left >= 4) {
			unsigned int n = min(left / 4, NUC970_SPI_WORDS);
			unsigned int i;

			for (i = 0; tx && i < n; i++)
				txw[i] = get_unaligned_be32(tx + 4 * i);
			ret = nuc970_spi_kick(hw, cntrl, 32, n,
					      tx ? txw : NULL,
					      rx ? rxw : NULL);
			if (ret)
				goto timeout;
			for (i = 0; rx && i < n; i++)
				put_unaligned_be32(rxw[i], rx + 4 * i);
			if (tx)
				tx += 4 * n;
			if (rx)
				rx += 4 * n;
			left -= 4 * n;
		}
	}

	while (left >= wbytes) {
		unsigned int n = min(left / wbytes, NUC970_SPI_WORDS);
		unsigned int i;

		for (i = 0; tx && i < n; i++) {
			if (wbytes == 1)
				txw[i] = tx[i];
			else if (wbytes == 2)
				txw[i] = ((const u16 *)tx)[i];
			else
				txw[i] = ((const u32 *)tx)[i];
		}
		ret = nuc970_spi_kick(hw, cntrl, bpw, n, tx ? txw : NULL,
				      rx ? rxw : NULL);
		if (ret)
			goto timeout;
		for (i = 0; rx && i < n; i++) {
			if (wbytes == 1)
				rx[i] = rxw[i];
			else if (wbytes == 2)
				((u16 *)rx)[i] = rxw[i];
			else
				((u32 *)rx)[i] = rxw[i];
		}
		if (tx)
			tx += n * wbytes;
		if (rx)
			rx += n * wbytes;
		left -= n * wbytes;
	}

	return 0;

timeout:
	dev_err(&host->dev, "transfer timed out\n");
	return ret;
}

static int nuc970_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *host;
	struct nuc970_spi *hw;

	host = devm_spi_alloc_host(dev, sizeof(*hw));
	if (!host)
		return -ENOMEM;
	hw = spi_controller_get_devdata(host);

	hw->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hw->base))
		return PTR_ERR(hw->base);

	hw->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(hw->clk))
		return dev_err_probe(dev, PTR_ERR(hw->clk), "no clock\n");

	/* polled operation: interrupt off, flag cleared, nothing selected */
	writel_relaxed(CNTRL_IF | CNTRL_TX_NEG, hw->base + REG_CNTRL);
	writel_relaxed(0, hw->base + REG_SSR);

	host->dev.of_node = dev->of_node;
	host->bus_num = -1;
	host->num_chipselect = NUC970_SPI_NUM_CS;
	host->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST |
			  SPI_TX_DUAL | SPI_RX_DUAL | SPI_TX_QUAD | SPI_RX_QUAD;
	host->bits_per_word_mask = SPI_BPW_RANGE_MASK(8, 32);
	host->min_speed_hz = DIV_ROUND_UP(clk_get_rate(hw->clk), 2 * 0x10000);
	host->max_speed_hz = clk_get_rate(hw->clk) / 2;
	host->set_cs = nuc970_spi_set_cs;
	host->transfer_one = nuc970_spi_transfer_one;

	return devm_spi_register_controller(dev, host);
}

static const struct of_device_id nuc970_spi_of_match[] = {
	{ .compatible = "nuvoton,nuc970-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_spi_of_match);

static struct platform_driver nuc970_spi_driver = {
	.probe	= nuc970_spi_probe,
	.driver	= {
		.name		= "nuc970-spi",
		.of_match_table	= nuc970_spi_of_match,
	},
};
module_platform_driver(nuc970_spi_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 SPI controller driver");
MODULE_LICENSE("GPL");
