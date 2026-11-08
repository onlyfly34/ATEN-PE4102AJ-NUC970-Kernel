// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Nuvoton NUC970/NUC977 GDMA (general DMA) controller
 *
 * Rewritten for the virt-dma framework from drivers/dma/nuc970_dma.c in
 * Nuvoton's NUC970 Linux BSP (Copyright (C) 2018 Nuvoton Technology Corp.,
 * SChung <schung@nuvoton.com>), with the register layout taken from the
 * BSP's arch/arm/mach-nuc970/include/mach/regs-gdma.h.
 *
 * The controller has two software-triggered memory-to-memory channels, each
 * with its own interrupt line; there are no peripheral request lines, so
 * this is a plain DMA_MEMCPY provider. The BSP also advertised DMA_SLAVE and
 * DMA_CYCLIC, but its implementations of those never programmed a device
 * address and could not work; they are not carried over.
 *
 * Differences from the BSP driver:
 *  - transfers longer than one TCNT (24 bits of transfer units) are split
 *    into chunks and run back to back from the interrupt handler;
 *  - the transfer width is chosen per chunk from the address/length
 *    alignment (32/16/8 bit) instead of always defaulting to 8 bit;
 *  - transfer-error interrupts are enabled and reported through the
 *    dmaengine result (the BSP masked the error flags out of its test).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "virt-dma.h"

/* Per-channel registers, channel n at 0x20 * n */
#define GDMA_CH_STRIDE		0x20
#define GDMA_CTL		0x00
#define  CTL_GDMAEN		BIT(0)	/* set to start, clears when done */
#define  CTL_TWS		GENMASK(13, 12)	/* 0: 8 bit, 1: 16 bit, 2: 32 bit */
#define  CTL_SOFTREQ		BIT(16)
#define GDMA_SRCB		0x04
#define GDMA_DSTB		0x08
#define GDMA_TCNT		0x0c	/* transfer count, in TWS units */
#define GDMA_CSRC		0x10
#define GDMA_CDST		0x14
#define GDMA_CTCNT		0x18
#define GDMA_DADR		0x1c

/* Shared interrupt control/status register */
#define GDMA_INTCS		0xa0
#define  INTCS_TCEN(ch)		BIT(2 * (ch))
#define  INTCS_TERREN(ch)	BIT(2 * (ch) + 1)
#define  INTCS_EN_MASK		GENMASK(3, 0)
#define  INTCS_TCF(ch)		BIT(8 + 2 * (ch))	/* write 1 to clear */
#define  INTCS_TERRF(ch)	BIT(9 + 2 * (ch))	/* write 1 to clear */
#define  INTCS_FLAG_MASK	GENMASK(11, 8)

#define GDMA_NR_CHANNELS	2
#define GDMA_TCNT_MAX		0xffffff
/* Largest chunk: fits TCNT at 8-bit width and keeps 4-byte alignment */
#define GDMA_MAX_CHUNK		(GDMA_TCNT_MAX & ~3U)

#define GDMA_BUSWIDTHS		(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |	\
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |	\
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

struct nuc970_gdma;

struct nuc970_gdma_desc {
	struct virt_dma_desc vd;
	dma_addr_t src;
	dma_addr_t dst;
	size_t len;
	size_t done;		/* bytes completed by earlier chunks */
};

struct nuc970_gdma_chan {
	struct virt_dma_chan vc;
	struct nuc970_gdma *gdma;
	void __iomem *regs;
	unsigned int id;
	int irq;
	struct nuc970_gdma_desc *desc;	/* in flight, protected by vc.lock */
	size_t chunk;			/* bytes in the running chunk */
};

struct nuc970_gdma {
	struct dma_device ddev;
	void __iomem *base;
	spinlock_t intcs_lock;		/* serialises INTCS updates */
	struct nuc970_gdma_chan chan[GDMA_NR_CHANNELS];
};

static inline struct nuc970_gdma_chan *to_gdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct nuc970_gdma_chan, vc.chan);
}

static inline struct nuc970_gdma_desc *to_gdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct nuc970_gdma_desc, vd);
}

/* Called with vc.lock held */
static void nuc970_gdma_start_chunk(struct nuc970_gdma_chan *c)
{
	struct nuc970_gdma_desc *d = c->desc;
	dma_addr_t src = d->src + d->done;
	dma_addr_t dst = d->dst + d->done;
	size_t rem = d->len - d->done;
	unsigned int shift;
	size_t bytes;
	u32 ctl;

	if (!((src | dst) & 3) && rem >= 4)
		shift = 2;
	else if (!((src | dst) & 1) && rem >= 2)
		shift = 1;
	else
		shift = 0;

	bytes = min_t(size_t, rem, GDMA_MAX_CHUNK);
	bytes &= ~(size_t)((1U << shift) - 1);
	c->chunk = bytes;

	ctl = FIELD_PREP(CTL_TWS, shift);
	writel(src, c->regs + GDMA_SRCB);
	writel(dst, c->regs + GDMA_DSTB);
	writel(bytes >> shift, c->regs + GDMA_TCNT);
	/* Same two-step sequence as the BSP: width first, then kick */
	writel(ctl, c->regs + GDMA_CTL);
	writel(ctl | CTL_GDMAEN | CTL_SOFTREQ, c->regs + GDMA_CTL);
}

/* Called with vc.lock held */
static void nuc970_gdma_start_next(struct nuc970_gdma_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);

	if (!vd) {
		c->desc = NULL;
		return;
	}

	list_del(&vd->node);
	c->desc = to_gdma_desc(vd);
	c->desc->done = 0;
	nuc970_gdma_start_chunk(c);
}

static irqreturn_t nuc970_gdma_irq(int irq, void *data)
{
	struct nuc970_gdma_chan *c = data;
	struct nuc970_gdma *gdma = c->gdma;
	u32 mine = INTCS_TCF(c->id) | INTCS_TERRF(c->id);
	struct nuc970_gdma_desc *d;
	u32 st;

	spin_lock(&gdma->intcs_lock);
	st = readl(gdma->base + GDMA_INTCS);
	if (st & mine)
		writel((st & INTCS_EN_MASK) | (st & mine),
		       gdma->base + GDMA_INTCS);
	spin_unlock(&gdma->intcs_lock);

	if (!(st & mine))
		return IRQ_NONE;

	spin_lock(&c->vc.lock);
	d = c->desc;
	if (!d)
		goto out;

	if (!(st & INTCS_TERRF(c->id))) {
		u32 ctl;

		/*
		 * GDMAEN clears itself when the transfer completes (the BSP's
		 * tx_status relies on that). Give it a few microseconds in case
		 * the flag is seen first; if it stays set, this is a stale flag
		 * (e.g. from a terminated transfer) and the running chunk will
		 * raise its own interrupt.
		 */
		if (readl_poll_timeout_atomic(c->regs + GDMA_CTL, ctl,
					      !(ctl & CTL_GDMAEN), 0, 10))
			goto out;
	}

	if (st & INTCS_TERRF(c->id)) {
		dev_err_ratelimited(c->gdma->ddev.dev,
				    "ch%u: transfer error at %#llx -> %#llx\n",
				    c->id, (u64)(d->src + d->done),
				    (u64)(d->dst + d->done));
		writel(0, c->regs + GDMA_CTL);
		d->vd.tx_result.result = DMA_TRANS_ABORTED;
		d->vd.tx_result.residue = d->len - d->done;
	} else {
		d->done += c->chunk;
		if (d->done < d->len) {
			nuc970_gdma_start_chunk(c);
			goto out;
		}
	}

	vchan_cookie_complete(&d->vd);
	nuc970_gdma_start_next(c);
out:
	spin_unlock(&c->vc.lock);
	return IRQ_HANDLED;
}

static struct dma_async_tx_descriptor *
nuc970_gdma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
			size_t len, unsigned long flags)
{
	struct nuc970_gdma_chan *c = to_gdma_chan(chan);
	struct nuc970_gdma_desc *d;

	if (!len)
		return NULL;

	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;

	d->src = src;
	d->dst = dst;
	d->len = len;

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static void nuc970_gdma_issue_pending(struct dma_chan *chan)
{
	struct nuc970_gdma_chan *c = to_gdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc) && !c->desc)
		nuc970_gdma_start_next(c);
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static enum dma_status nuc970_gdma_tx_status(struct dma_chan *chan,
					     dma_cookie_t cookie,
					     struct dma_tx_state *state)
{
	struct nuc970_gdma_chan *c = to_gdma_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;
	size_t residue = 0;

	ret = dma_cookie_status(chan, cookie, state);
	if (ret == DMA_COMPLETE || !state)
		return ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	vd = vchan_find_desc(&c->vc, cookie);
	if (vd)
		residue = to_gdma_desc(vd)->len;
	else if (c->desc && c->desc->vd.tx.cookie == cookie)
		residue = c->desc->len - c->desc->done;
	spin_unlock_irqrestore(&c->vc.lock, flags);

	dma_set_residue(state, residue);
	return ret;
}

static int nuc970_gdma_terminate_all(struct dma_chan *chan)
{
	struct nuc970_gdma_chan *c = to_gdma_chan(chan);
	struct nuc970_gdma *gdma = c->gdma;
	unsigned long flags;
	u32 st;
	LIST_HEAD(head);

	spin_lock_irqsave(&c->vc.lock, flags);

	writel(0, c->regs + GDMA_CTL);

	spin_lock(&gdma->intcs_lock);
	st = readl(gdma->base + GDMA_INTCS);
	writel((st & INTCS_EN_MASK) | INTCS_TCF(c->id) | INTCS_TERRF(c->id),
	       gdma->base + GDMA_INTCS);
	spin_unlock(&gdma->intcs_lock);

	if (c->desc) {
		vchan_terminate_vdesc(&c->desc->vd);
		c->desc = NULL;
	}
	vchan_get_all_descriptors(&c->vc, &head);

	spin_unlock_irqrestore(&c->vc.lock, flags);

	vchan_dma_desc_free_list(&c->vc, &head);
	return 0;
}

static void nuc970_gdma_synchronize(struct dma_chan *chan)
{
	struct nuc970_gdma_chan *c = to_gdma_chan(chan);

	synchronize_irq(c->irq);
	vchan_synchronize(&c->vc);
}

static void nuc970_gdma_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(to_virt_chan(chan));
}

static void nuc970_gdma_desc_free(struct virt_dma_desc *vd)
{
	kfree(to_gdma_desc(vd));
}

static void nuc970_gdma_shutdown(void *data)
{
	struct nuc970_gdma *gdma = data;
	unsigned int i;

	for (i = 0; i < GDMA_NR_CHANNELS; i++) {
		writel(0, gdma->chan[i].regs + GDMA_CTL);
		tasklet_kill(&gdma->chan[i].vc.task);
	}
	writel(INTCS_FLAG_MASK, gdma->base + GDMA_INTCS);
}

static void nuc970_gdma_of_dma_free(void *data)
{
	of_dma_controller_free(data);
}

static void nuc970_gdma_proxy_unregister(void *data)
{
	platform_device_unregister(data);
}

static int nuc970_gdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_gdma *gdma;
	struct dma_device *dd;
	struct clk *clk;
	unsigned int i;
	int ret;

	gdma = devm_kzalloc(dev, sizeof(*gdma), GFP_KERNEL);
	if (!gdma)
		return -ENOMEM;

	gdma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdma->base))
		return PTR_ERR(gdma->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "cannot enable clock\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	spin_lock_init(&gdma->intcs_lock);

	dd = &gdma->ddev;
	INIT_LIST_HEAD(&dd->channels);
	dma_cap_set(DMA_MEMCPY, dd->cap_mask);
	dd->dev = dev;
	dd->device_prep_dma_memcpy = nuc970_gdma_prep_memcpy;
	dd->device_issue_pending = nuc970_gdma_issue_pending;
	dd->device_tx_status = nuc970_gdma_tx_status;
	dd->device_terminate_all = nuc970_gdma_terminate_all;
	dd->device_synchronize = nuc970_gdma_synchronize;
	dd->device_free_chan_resources = nuc970_gdma_free_chan_resources;
	dd->src_addr_widths = GDMA_BUSWIDTHS;
	dd->dst_addr_widths = GDMA_BUSWIDTHS;
	dd->directions = BIT(DMA_MEM_TO_MEM);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	dma_set_max_seg_size(dev, UINT_MAX);

	/* Quiesce both channels, drop stale flags */
	for (i = 0; i < GDMA_NR_CHANNELS; i++)
		writel(0, gdma->base + i * GDMA_CH_STRIDE + GDMA_CTL);
	writel(INTCS_FLAG_MASK, gdma->base + GDMA_INTCS);

	for (i = 0; i < GDMA_NR_CHANNELS; i++) {
		struct nuc970_gdma_chan *c = &gdma->chan[i];
		const char *name;

		c->gdma = gdma;
		c->id = i;
		c->regs = gdma->base + i * GDMA_CH_STRIDE;
		c->vc.desc_free = nuc970_gdma_desc_free;
		vchan_init(&c->vc, dd);

		c->irq = platform_get_irq(pdev, i);
		if (c->irq < 0)
			return c->irq;

		name = devm_kasprintf(dev, GFP_KERNEL, "%s:ch%u",
				      dev_name(dev), i);
		if (!name)
			return -ENOMEM;

		ret = devm_request_irq(dev, c->irq, nuc970_gdma_irq, 0, name, c);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request irq %d\n",
					     c->irq);
	}

	/* Runs after the dmaengine device is unregistered (devm is LIFO) */
	ret = devm_add_action_or_reset(dev, nuc970_gdma_shutdown, gdma);
	if (ret)
		return ret;

	writel(INTCS_FLAG_MASK | INTCS_TCEN(0) | INTCS_TERREN(0) |
	       INTCS_TCEN(1) | INTCS_TERREN(1), gdma->base + GDMA_INTCS);

	ret = dmaenginem_async_device_register(dd);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register dmaengine\n");

	if (dev->of_node) {
		ret = of_dma_controller_register(dev->of_node,
						 of_dma_xlate_by_chan_id, dd);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot register OF DMA controller\n");
		ret = devm_add_action_or_reset(dev, nuc970_gdma_of_dma_free,
					       dev->of_node);
		if (ret)
			return ret;
	}

	/*
	 * The vendor /dev/dma_proxy is a pure software user of a memcpy
	 * channel, not hardware, so it gets no DT node: instantiate it here,
	 * after the channels are registered, so that its probe can always
	 * find one.
	 */
	if (IS_ENABLED(CONFIG_NUC970_DMA_PROXY)) {
		struct platform_device *proxy;

		proxy = platform_device_register_data(dev, "nuc970-dma-proxy",
						      PLATFORM_DEVID_NONE,
						      NULL, 0);
		if (IS_ERR(proxy)) {
			dev_warn(dev, "cannot create dma_proxy device: %pe\n",
				 proxy);
		} else {
			ret = devm_add_action_or_reset(dev,
						       nuc970_gdma_proxy_unregister,
						       proxy);
			if (ret)
				return ret;
		}
	}

	dev_info(dev, "NUC970 GDMA, %d memcpy channels\n", GDMA_NR_CHANNELS);
	return 0;
}

static const struct of_device_id nuc970_gdma_of_match[] = {
	{ .compatible = "nuvoton,nuc970-dma" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_gdma_of_match);

static struct platform_driver nuc970_gdma_driver = {
	.probe = nuc970_gdma_probe,
	.driver = {
		.name = "nuc970-gdma",
		.of_match_table = nuc970_gdma_of_match,
		/* unbinding a dmaengine provider under its clients is unsafe */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(nuc970_gdma_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 GDMA memcpy engine");
MODULE_LICENSE("GPL");
