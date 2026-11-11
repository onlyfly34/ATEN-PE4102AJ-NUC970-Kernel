// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 Ethernet MAC (EMAC) driver
 *
 * Rewritten from drivers/net/ethernet/nuvoton/nuc970_ether0.c in Nuvoton's
 * NUC970 Linux BSP (Copyright (c) 2014 Nuvoton technology corporation) for
 * the current networking core: DT resources and clocks, phylib with an
 * optional DT MDIO node, NAPI receive, and no hard-coded register mappings.
 * PTP time stamping and NC-SI support of the BSP are not ported.
 *
 * The MAC uses two rings of 16-byte descriptors chained through their "next"
 * field; bit 31 of the first word hands a descriptor to the DMA engine.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>

#define DRV_NAME		"nuc970-emac"

/* registers */
#define REG_CAMCMR		0x000
#define  CAMCMR_AUP		BIT(0)
#define  CAMCMR_AMP		BIT(1)
#define  CAMCMR_ABP		BIT(2)
#define  CAMCMR_ECMP		BIT(4)
#define REG_CAMEN		0x004
#define REG_CAMM(n)		(0x008 + 8 * (n))
#define REG_CAML(n)		(0x00c + 8 * (n))
#define REG_TXDLSA		0x088
#define REG_RXDLSA		0x08c
#define REG_MCMDR		0x090
#define  MCMDR_RXON		BIT(0)
#define  MCMDR_ALP		BIT(1)
#define  MCMDR_ACP		BIT(3)
#define  MCMDR_SPCRC		BIT(5)
#define  MCMDR_TXON		BIT(8)
#define  MCMDR_FDUP		BIT(18)
#define  MCMDR_OPMOD		BIT(20)		/* 100 Mbit/s */
#define  MCMDR_SWR		BIT(24)
#define REG_MIID		0x094
#define REG_MIIDA		0x098
#define  MIIDA_PHYWR		BIT(16)
#define  MIIDA_PHYBUSY		BIT(17)
#define  MIIDA_MDCON		BIT(19)
#define REG_FFTCR		0x09c
#define  FFTCR_TXTHD		(0x3 << 8)
#define  FFTCR_BLENGTH		(0x1 << 20)
#define REG_TSDR		0x0a0
#define REG_RSDR		0x0a4
#define REG_DMARFC		0x0a8
#define REG_MIEN		0x0ac
#define REG_MISTA		0x0b0
#define  MI_RXINTR		BIT(0)
#define  MI_RXGD		BIT(4)
#define  MI_RDU			BIT(10)
#define  MI_RXBERR		BIT(11)
#define  MI_TXINTR		BIT(16)
#define  MI_TXCP		BIT(18)
#define  MI_EXDEF		BIT(19)
#define  MI_TXABT		BIT(21)
#define  MI_TXBERR		BIT(24)
#define  MI_RX_MASK		0x0000ffff
#define  MI_TX_MASK		0xffff0000

/* descriptors */
#define DESC_OWN_DMA		BIT(31)
#define TXD_PADEN		BIT(0)
#define TXD_CRCAPP		BIT(1)
#define TXD_INTEN		BIT(2)
#define TXDS_TXCP		BIT(19)
#define RXDS_CRCE		BIT(17)
#define RXDS_PTLE		BIT(19)
#define RXDS_RXGD		BIT(20)
#define RXDS_ALIE		BIT(21)
#define RXDS_RP			BIT(22)
#define DS_LEN_MASK		0xffff

#define TX_RING_SIZE		32
#define RX_RING_SIZE		32
#define RX_BUF_SIZE		1536	/* also the DMA receive limit */
#define MDIO_TIMEOUT_US		10000
#define TX_TIMEOUT		(2 * HZ)

struct nuc970_rxd {
	u32 sl;		/* status, length, ownership */
	u32 buffer;
	u32 reserved;
	u32 next;
};

struct nuc970_txd {
	u32 mode;	/* ownership, control */
	u32 buffer;
	u32 sl;		/* status, length */
	u32 next;
};

struct nuc970_emac {
	struct net_device *ndev;
	struct device *dev;
	void __iomem *base;
	struct clk *hclk;
	struct clk *eclk;
	int txirq;
	int rxirq;

	struct nuc970_txd *txd;
	struct nuc970_rxd *rxd;
	dma_addr_t txd_dma;
	dma_addr_t rxd_dma;
	struct sk_buff *tx_skb[TX_RING_SIZE];
	dma_addr_t tx_dma[TX_RING_SIZE];
	struct sk_buff *rx_skb[RX_RING_SIZE];
	dma_addr_t rx_dma[RX_RING_SIZE];
	unsigned int cur_tx;
	unsigned int dirty_tx;
	unsigned int cur_rx;
	spinlock_t tx_lock;

	struct napi_struct napi;
	struct mii_bus *mii;
	phy_interface_t phy_mode;
	int speed;
	int duplex;
	int link;
};

static char *ethaddr0;
/* BSP compatibility: "ethaddr0=xx:xx:xx:xx:xx:xx" on the command line */
static int __init nuc970_emac_ethaddr0(char *str)
{
	ethaddr0 = str;
	return 1;
}
__setup("ethaddr0=", nuc970_emac_ethaddr0);

static inline u32 emac_read(struct nuc970_emac *priv, unsigned int reg)
{
	return readl_relaxed(priv->base + reg);
}

static inline void emac_write(struct nuc970_emac *priv, unsigned int reg,
			      u32 val)
{
	writel_relaxed(val, priv->base + reg);
}

static void emac_rmw(struct nuc970_emac *priv, unsigned int reg, u32 clr,
		     u32 set)
{
	emac_write(priv, reg, (emac_read(priv, reg) & ~clr) | set);
}

/* MDIO */

static int nuc970_mdio_wait(struct nuc970_emac *priv)
{
	u32 val;

	return readl_relaxed_poll_timeout(priv->base + REG_MIIDA, val,
					  !(val & MIIDA_PHYBUSY), 1,
					  MDIO_TIMEOUT_US);
}

static int nuc970_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct nuc970_emac *priv = bus->priv;
	int ret;

	emac_write(priv, REG_MIIDA, (addr << 8) | regnum | MIIDA_PHYBUSY |
		   MIIDA_MDCON);
	ret = nuc970_mdio_wait(priv);
	if (ret)
		return ret;
	return emac_read(priv, REG_MIID) & 0xffff;
}

static int nuc970_mdio_write(struct mii_bus *bus, int addr, int regnum,
			     u16 val)
{
	struct nuc970_emac *priv = bus->priv;

	emac_write(priv, REG_MIID, val);
	emac_write(priv, REG_MIIDA, (addr << 8) | regnum | MIIDA_PHYBUSY |
		   MIIDA_MDCON | MIIDA_PHYWR);
	return nuc970_mdio_wait(priv);
}

/* MAC helpers */

static void nuc970_write_cam(struct nuc970_emac *priv, unsigned int n,
			     const u8 *addr)
{
	emac_write(priv, REG_CAML(n), (addr[4] << 24) | (addr[5] << 16));
	emac_write(priv, REG_CAMM(n), (addr[0] << 24) | (addr[1] << 16) |
		   (addr[2] << 8) | addr[3]);
}

static bool nuc970_read_cam(struct nuc970_emac *priv, u8 *addr)
{
	u32 m = emac_read(priv, REG_CAMM(0));
	u32 l = emac_read(priv, REG_CAML(0));

	addr[0] = m >> 24;
	addr[1] = m >> 16;
	addr[2] = m >> 8;
	addr[3] = m;
	addr[4] = l >> 24;
	addr[5] = l >> 16;
	return is_valid_ether_addr(addr);
}

static void nuc970_soft_reset(struct nuc970_emac *priv)
{
	u32 val;

	emac_rmw(priv, REG_MCMDR, MCMDR_TXON | MCMDR_RXON, 0);
	emac_rmw(priv, REG_MCMDR, 0, MCMDR_SWR);
	if (readl_relaxed_poll_timeout(priv->base + REG_MCMDR, val,
				       !(val & MCMDR_SWR), 10, 10000))
		netdev_warn(priv->ndev, "MAC reset did not complete\n");
}

static void nuc970_set_opmode(struct nuc970_emac *priv)
{
	u32 val = emac_read(priv, REG_MCMDR) & ~(MCMDR_OPMOD | MCMDR_FDUP);

	if (priv->speed == SPEED_100)
		val |= MCMDR_OPMOD;
	if (priv->duplex == DUPLEX_FULL)
		val |= MCMDR_FDUP;
	emac_write(priv, REG_MCMDR, val);
	/* restart TX in case frames were queued while the link was down */
	emac_write(priv, REG_TSDR, 1);
}

static void nuc970_adjust_link(struct net_device *ndev)
{
	struct nuc970_emac *priv = netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	bool changed = false;

	if (phydev->link) {
		if (priv->speed != phydev->speed ||
		    priv->duplex != phydev->duplex) {
			priv->speed = phydev->speed;
			priv->duplex = phydev->duplex;
			changed = true;
		}
	}
	if (phydev->link != priv->link) {
		priv->link = phydev->link;
		changed = true;
	}

	if (changed) {
		if (phydev->link)
			nuc970_set_opmode(priv);
		phy_print_status(phydev);
	}
}

/* rings */

static int nuc970_rx_refill(struct nuc970_emac *priv, unsigned int i)
{
	struct sk_buff *skb;
	dma_addr_t dma;

	skb = netdev_alloc_skb_ip_align(priv->ndev, RX_BUF_SIZE);
	if (!skb)
		return -ENOMEM;

	dma = dma_map_single(priv->dev, skb->data, RX_BUF_SIZE,
			     DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->dev, dma)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}

	priv->rx_skb[i] = skb;
	priv->rx_dma[i] = dma;
	WRITE_ONCE(priv->rxd[i].buffer, dma);
	return 0;
}

static void nuc970_free_rings(struct nuc970_emac *priv)
{
	unsigned int i;

	for (i = 0; i < TX_RING_SIZE; i++) {
		if (!priv->tx_skb[i])
			continue;
		dma_unmap_single(priv->dev, priv->tx_dma[i],
				 priv->tx_skb[i]->len, DMA_TO_DEVICE);
		dev_kfree_skb_any(priv->tx_skb[i]);
		priv->tx_skb[i] = NULL;
	}
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (!priv->rx_skb[i])
			continue;
		dma_unmap_single(priv->dev, priv->rx_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		dev_kfree_skb_any(priv->rx_skb[i]);
		priv->rx_skb[i] = NULL;
	}

	if (priv->txd)
		dma_free_coherent(priv->dev, sizeof(*priv->txd) * TX_RING_SIZE,
				  priv->txd, priv->txd_dma);
	if (priv->rxd)
		dma_free_coherent(priv->dev, sizeof(*priv->rxd) * RX_RING_SIZE,
				  priv->rxd, priv->rxd_dma);
	priv->txd = NULL;
	priv->rxd = NULL;
}

static int nuc970_alloc_rings(struct nuc970_emac *priv)
{
	unsigned int i;

	priv->txd = dma_alloc_coherent(priv->dev,
				       sizeof(*priv->txd) * TX_RING_SIZE,
				       &priv->txd_dma, GFP_KERNEL);
	priv->rxd = dma_alloc_coherent(priv->dev,
				       sizeof(*priv->rxd) * RX_RING_SIZE,
				       &priv->rxd_dma, GFP_KERNEL);
	if (!priv->txd || !priv->rxd)
		goto err;

	for (i = 0; i < TX_RING_SIZE; i++) {
		priv->txd[i].mode = TXD_PADEN | TXD_CRCAPP | TXD_INTEN;
		priv->txd[i].buffer = 0;
		priv->txd[i].sl = 0;
		priv->txd[i].next = priv->txd_dma +
			sizeof(*priv->txd) * ((i + 1) % TX_RING_SIZE);
	}

	for (i = 0; i < RX_RING_SIZE; i++) {
		priv->rxd[i].next = priv->rxd_dma +
			sizeof(*priv->rxd) * ((i + 1) % RX_RING_SIZE);
		if (nuc970_rx_refill(priv, i))
			goto err;
		priv->rxd[i].sl = DESC_OWN_DMA;
	}

	priv->cur_tx = 0;
	priv->dirty_tx = 0;
	priv->cur_rx = 0;
	return 0;

err:
	nuc970_free_rings(priv);
	return -ENOMEM;
}

static void nuc970_init_mac(struct nuc970_emac *priv)
{
	struct net_device *ndev = priv->ndev;
	u32 mcmdr = MCMDR_SPCRC | MCMDR_ACP;

	nuc970_soft_reset(priv);

	emac_write(priv, REG_FFTCR, FFTCR_TXTHD | FFTCR_BLENGTH);
	emac_write(priv, REG_RXDLSA, priv->rxd_dma);
	emac_write(priv, REG_TXDLSA, priv->txd_dma);

	nuc970_write_cam(priv, 0, ndev->dev_addr);
	emac_write(priv, REG_CAMEN, emac_read(priv, REG_CAMEN) | BIT(0));
	emac_write(priv, REG_CAMCMR, CAMCMR_ECMP | CAMCMR_ABP | CAMCMR_AMP);

	emac_write(priv, REG_MIEN, MI_RXINTR | MI_RXGD | MI_RDU | MI_RXBERR |
		   MI_TXINTR | MI_TXCP | MI_TXABT | MI_TXBERR);
	emac_write(priv, REG_MISTA, 0xffffffff);

	/* the RX DMA must never write past a buffer */
	emac_write(priv, REG_DMARFC, RX_BUF_SIZE);
	if (IS_ENABLED(CONFIG_VLAN_8021Q))
		mcmdr |= MCMDR_ALP;
	if (priv->speed == SPEED_100)
		mcmdr |= MCMDR_OPMOD;
	if (priv->duplex == DUPLEX_FULL)
		mcmdr |= MCMDR_FDUP;
	emac_write(priv, REG_MCMDR, mcmdr | MCMDR_TXON);
}

/* data path */

static netdev_tx_t nuc970_start_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct nuc970_emac *priv = netdev_priv(ndev);
	unsigned int entry;
	struct nuc970_txd *txd;
	unsigned long flags;
	dma_addr_t dma;

	if (skb_put_padto(skb, ETH_ZLEN)) {
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	dma = dma_map_single(priv->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, dma)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&priv->tx_lock, flags);

	entry = priv->cur_tx;
	txd = &priv->txd[entry];
	if (priv->tx_skb[entry] || (READ_ONCE(txd->mode) & DESC_OWN_DMA)) {
		/* should not happen: the queue is stopped when the ring fills */
		netif_stop_queue(ndev);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		dma_unmap_single(priv->dev, dma, skb->len, DMA_TO_DEVICE);
		return NETDEV_TX_BUSY;
	}

	priv->tx_skb[entry] = skb;
	priv->tx_dma[entry] = dma;
	WRITE_ONCE(txd->buffer, dma);
	WRITE_ONCE(txd->sl, skb->len);
	dma_wmb();
	WRITE_ONCE(txd->mode, TXD_PADEN | TXD_CRCAPP | TXD_INTEN |
		   DESC_OWN_DMA);
	wmb();
	emac_write(priv, REG_TSDR, 1);

	skb_tx_timestamp(skb);

	priv->cur_tx = (entry + 1) % TX_RING_SIZE;
	if (priv->tx_skb[priv->cur_tx])
		netif_stop_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);
	return NETDEV_TX_OK;
}

static irqreturn_t nuc970_tx_irq(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct nuc970_emac *priv = netdev_priv(ndev);
	u32 status;

	status = emac_read(priv, REG_MISTA) & MI_TX_MASK;
	emac_write(priv, REG_MISTA, status);

	spin_lock(&priv->tx_lock);
	while (priv->tx_skb[priv->dirty_tx]) {
		unsigned int entry = priv->dirty_tx;
		struct nuc970_txd *txd = &priv->txd[entry];
		struct sk_buff *skb = priv->tx_skb[entry];
		u32 sl;

		if (READ_ONCE(txd->mode) & DESC_OWN_DMA)
			break;
		dma_rmb();
		sl = READ_ONCE(txd->sl);

		dma_unmap_single(priv->dev, priv->tx_dma[entry], skb->len,
				 DMA_TO_DEVICE);
		if (sl & TXDS_TXCP) {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
		} else {
			ndev->stats.tx_errors++;
		}
		dev_consume_skb_irq(skb);
		priv->tx_skb[entry] = NULL;
		priv->dirty_tx = (entry + 1) % TX_RING_SIZE;
	}
	if (netif_queue_stopped(ndev) && !priv->tx_skb[priv->cur_tx])
		netif_wake_queue(ndev);
	spin_unlock(&priv->tx_lock);

	if (status & MI_EXDEF)
		netdev_warn_once(ndev, "TX defer limit exceeded\n");
	if (status & MI_TXBERR)
		netdev_err(ndev, "TX bus error (MISTA %#x)\n", status);

	return IRQ_HANDLED;
}

static int nuc970_poll(struct napi_struct *napi, int budget)
{
	struct nuc970_emac *priv = container_of(napi, struct nuc970_emac, napi);
	struct net_device *ndev = priv->ndev;
	unsigned int max_len = IS_ENABLED(CONFIG_VLAN_8021Q) ?
			       RX_BUF_SIZE : ETH_FRAME_LEN;
	int done = 0;

	while (done < budget) {
		unsigned int entry = priv->cur_rx;
		struct nuc970_rxd *rxd = &priv->rxd[entry];
		struct sk_buff *skb;
		dma_addr_t dma;
		u32 sl, len;

		sl = READ_ONCE(rxd->sl);
		if (sl & DESC_OWN_DMA)
			break;
		dma_rmb();
		len = sl & DS_LEN_MASK;

		if (likely((sl & RXDS_RXGD) && len <= max_len)) {
			skb = priv->rx_skb[entry];
			dma = priv->rx_dma[entry];
			if (nuc970_rx_refill(priv, entry)) {
				/* keep the old buffer, drop this frame */
				ndev->stats.rx_dropped++;
			} else {
				dma_unmap_single(priv->dev, dma, RX_BUF_SIZE,
						 DMA_FROM_DEVICE);
				skb_put(skb, len);
				skb->protocol = eth_type_trans(skb, ndev);
				napi_gro_receive(napi, skb);
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
			}
		} else {
			ndev->stats.rx_errors++;
			if (sl & RXDS_RP)
				ndev->stats.rx_length_errors++;
			else if (sl & RXDS_CRCE)
				ndev->stats.rx_crc_errors++;
			else if (sl & RXDS_ALIE)
				ndev->stats.rx_frame_errors++;
			else if (sl & RXDS_PTLE)
				ndev->stats.rx_over_errors++;
		}

		dma_wmb();
		WRITE_ONCE(rxd->sl, DESC_OWN_DMA);
		priv->cur_rx = (entry + 1) % RX_RING_SIZE;
		done++;
	}

	/* restart RX DMA in case it ran out of descriptors */
	wmb();
	emac_write(priv, REG_RSDR, 1);

	if (done < budget && napi_complete_done(napi, done))
		emac_rmw(priv, REG_MIEN, 0, MI_RXINTR);

	return done;
}

static irqreturn_t nuc970_rx_irq(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct nuc970_emac *priv = netdev_priv(ndev);
	u32 status;

	status = emac_read(priv, REG_MISTA) & MI_RX_MASK;
	emac_write(priv, REG_MISTA, status);

	if (unlikely(status & MI_RXBERR))
		netdev_err(ndev, "RX bus error (MISTA %#x)\n", status);

	if (status & (MI_RXGD | MI_RDU)) {
		emac_rmw(priv, REG_MIEN, MI_RXINTR, 0);
		napi_schedule(&priv->napi);
	}
	return IRQ_HANDLED;
}

/* net_device ops */

static int nuc970_open(struct net_device *ndev)
{
	struct nuc970_emac *priv = netdev_priv(ndev);
	int ret;

	ret = nuc970_alloc_rings(priv);
	if (ret)
		return ret;

	nuc970_init_mac(priv);

	ret = request_irq(priv->txirq, nuc970_tx_irq, 0, "emac-tx", ndev);
	if (ret)
		goto err_rings;
	ret = request_irq(priv->rxirq, nuc970_rx_irq, 0, "emac-rx", ndev);
	if (ret)
		goto err_txirq;

	napi_enable(&priv->napi);
	netif_start_queue(ndev);
	emac_rmw(priv, REG_MCMDR, 0, MCMDR_RXON);
	emac_write(priv, REG_RSDR, 1);

	priv->link = 0;
	phy_start(ndev->phydev);
	return 0;

err_txirq:
	free_irq(priv->txirq, ndev);
err_rings:
	nuc970_soft_reset(priv);
	nuc970_free_rings(priv);
	return ret;
}

static int nuc970_stop(struct net_device *ndev)
{
	struct nuc970_emac *priv = netdev_priv(ndev);

	phy_stop(ndev->phydev);
	netif_stop_queue(ndev);
	napi_disable(&priv->napi);

	emac_write(priv, REG_MIEN, 0);
	nuc970_soft_reset(priv);

	free_irq(priv->rxirq, ndev);
	free_irq(priv->txirq, ndev);
	nuc970_free_rings(priv);
	return 0;
}

static void nuc970_set_rx_mode(struct net_device *ndev)
{
	struct nuc970_emac *priv = netdev_priv(ndev);
	u32 mode = CAMCMR_ECMP | CAMCMR_ABP;

	if (ndev->flags & IFF_PROMISC)
		mode |= CAMCMR_AUP | CAMCMR_AMP;
	else if ((ndev->flags & IFF_ALLMULTI) || !netdev_mc_empty(ndev))
		mode |= CAMCMR_AMP;
	emac_write(priv, REG_CAMCMR, mode);
}

static int nuc970_set_mac_address(struct net_device *ndev, void *addr)
{
	struct nuc970_emac *priv = netdev_priv(ndev);
	int ret;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;
	nuc970_write_cam(priv, 0, ndev->dev_addr);
	return 0;
}

static void nuc970_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct nuc970_emac *priv = netdev_priv(ndev);

	netdev_warn(ndev, "TX timeout, kicking DMA\n");
	ndev->stats.tx_errors++;
	emac_write(priv, REG_TSDR, 1);
	netif_wake_queue(ndev);
}

static const struct net_device_ops nuc970_netdev_ops = {
	.ndo_open		= nuc970_open,
	.ndo_stop		= nuc970_stop,
	.ndo_start_xmit		= nuc970_start_xmit,
	.ndo_set_rx_mode	= nuc970_set_rx_mode,
	.ndo_set_mac_address	= nuc970_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_eth_ioctl		= phy_do_ioctl_running,
	.ndo_tx_timeout		= nuc970_tx_timeout,
};

static void nuc970_get_drvinfo(struct net_device *ndev,
			       struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRV_NAME, sizeof(info->driver));
	strscpy(info->bus_info, dev_name(ndev->dev.parent),
		sizeof(info->bus_info));
}

static const struct ethtool_ops nuc970_ethtool_ops = {
	.get_drvinfo		= nuc970_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.nway_reset		= phy_ethtool_nway_reset,
};

/* probe */

static void nuc970_get_mac(struct nuc970_emac *priv)
{
	struct net_device *ndev = priv->ndev;
	u8 addr[ETH_ALEN];

	if (!of_get_ethdev_address(priv->dev->of_node, ndev))
		return;

	if (ethaddr0 && mac_pton(ethaddr0, addr) &&
	    is_valid_ether_addr(addr)) {
		eth_hw_addr_set(ndev, addr);
		return;
	}

	/* the boot loader may have programmed CAM0 */
	if (nuc970_read_cam(priv, addr)) {
		eth_hw_addr_set(ndev, addr);
		return;
	}

	eth_hw_addr_random(ndev);
	dev_warn(priv->dev, "no MAC address, using random %pM\n",
		 ndev->dev_addr);
}

static int nuc970_mdio_init(struct nuc970_emac *priv)
{
	struct device_node *np = priv->dev->of_node;
	struct device_node *mdio_np;
	int ret;

	priv->mii = devm_mdiobus_alloc(priv->dev);
	if (!priv->mii)
		return -ENOMEM;

	priv->mii->name = "nuc970-emac-mdio";
	priv->mii->read = nuc970_mdio_read;
	priv->mii->write = nuc970_mdio_write;
	priv->mii->priv = priv;
	priv->mii->parent = priv->dev;
	snprintf(priv->mii->id, MII_BUS_ID_SIZE, "%s", dev_name(priv->dev));

	mdio_np = of_get_child_by_name(np, "mdio");
	ret = devm_of_mdiobus_register(priv->dev, priv->mii, mdio_np);
	of_node_put(mdio_np);
	return ret;
}

static int nuc970_phy_connect(struct nuc970_emac *priv)
{
	struct net_device *ndev = priv->ndev;
	struct device_node *phy_np;
	struct phy_device *phydev;

	phy_np = of_parse_phandle(priv->dev->of_node, "phy-handle", 0);
	if (phy_np) {
		phydev = of_phy_connect(ndev, phy_np, nuc970_adjust_link, 0,
					priv->phy_mode);
		of_node_put(phy_np);
		if (!phydev)
			return -EPROBE_DEFER;
	} else {
		/* like the BSP: use the first PHY found on the bus */
		phydev = phy_find_first(priv->mii);
		if (!phydev) {
			dev_err(priv->dev, "no PHY found\n");
			return -ENODEV;
		}
		phydev = phy_connect(ndev, phydev_name(phydev),
				     nuc970_adjust_link, priv->phy_mode);
		if (IS_ERR(phydev))
			return PTR_ERR(phydev);
	}

	/* 10/100 MAC without EEE (LPI) support */
	phy_set_max_speed(phydev, SPEED_100);
	phy_disable_eee(phydev);
	phy_attached_info(phydev);
	return 0;
}

static int nuc970_emac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_emac *priv;
	struct net_device *ndev;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = dev;
	priv->speed = SPEED_100;
	priv->duplex = DUPLEX_FULL;
	spin_lock_init(&priv->tx_lock);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->txirq = platform_get_irq_byname(pdev, "tx");
	if (priv->txirq < 0)
		return priv->txirq;
	priv->rxirq = platform_get_irq_byname(pdev, "rx");
	if (priv->rxirq < 0)
		return priv->rxirq;

	/* both gates share HCLKEN[16]; the engine clock divider sets MDC */
	priv->hclk = devm_clk_get_enabled(dev, "hclk");
	if (IS_ERR(priv->hclk))
		return dev_err_probe(dev, PTR_ERR(priv->hclk), "no hclk\n");
	priv->eclk = devm_clk_get_enabled(dev, "eclk");
	if (IS_ERR(priv->eclk))
		return dev_err_probe(dev, PTR_ERR(priv->eclk), "no eclk\n");
	/* MDC = 1 MHz, as in the BSP */
	if (clk_set_rate(priv->eclk, 1000000))
		dev_warn(dev, "cannot set MDC clock to 1 MHz\n");

	ret = of_get_phy_mode(dev->of_node, &priv->phy_mode);
	if (ret)
		priv->phy_mode = PHY_INTERFACE_MODE_RMII;

	nuc970_get_mac(priv);

	ndev->netdev_ops = &nuc970_netdev_ops;
	ndev->ethtool_ops = &nuc970_ethtool_ops;
	ndev->watchdog_timeo = TX_TIMEOUT;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_DATA_LEN;
	netif_napi_add(ndev, &priv->napi, nuc970_poll);

	ret = nuc970_mdio_init(priv);
	if (ret)
		goto err_napi;

	ret = nuc970_phy_connect(priv);
	if (ret)
		goto err_napi;

	ret = register_netdev(ndev);
	if (ret)
		goto err_phy;

	netdev_info(ndev, "NUC970 EMAC at %pR, MAC %pM\n",
		    platform_get_resource(pdev, IORESOURCE_MEM, 0),
		    ndev->dev_addr);
	return 0;

err_phy:
	phy_disconnect(ndev->phydev);
err_napi:
	netif_napi_del(&priv->napi);
	return ret;
}

static void nuc970_emac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct nuc970_emac *priv = netdev_priv(ndev);

	unregister_netdev(ndev);
	phy_disconnect(ndev->phydev);
	netif_napi_del(&priv->napi);
}

static const struct of_device_id nuc970_emac_of_match[] = {
	{ .compatible = "nuvoton,nuc970-emac" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_emac_of_match);

static struct platform_driver nuc970_emac_driver = {
	.probe	= nuc970_emac_probe,
	.remove	= nuc970_emac_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= nuc970_emac_of_match,
	},
};
module_platform_driver(nuc970_emac_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 Ethernet MAC driver");
MODULE_LICENSE("GPL");
