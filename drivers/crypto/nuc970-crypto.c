// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Nuvoton NUC970/NUC977 cryptographic accelerator - AES engine
 *
 * Rewritten from drivers/crypto/nuc970-crypto.c in Nuvoton's NUC970 Linux
 * BSPs (3.10.108 and 5.10; Copyright (c) Nuvoton Technology Corporation)
 * for the 6.18 skcipher + crypto_engine API.
 *
 * Only the AES modes that the kernel self-tests can verify are registered
 * (ecb, cbc, ctr). Every DMA chunk is programmed with a software-computed
 * IV, so correctness does not depend on the engine's cascade/feedback
 * behaviour or on the width of its CTR counter. The BSP's DES/3DES,
 * cfb/ofb/cbc-cs, SHA/HMAC and MTP key-storage interfaces are not ported
 * here.
 */

#include <crypto/aes.h>
#include <crypto/engine.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#define CRPT_INTEN		0x000
#define CRPT_INTSTS		0x004
#define  INT_AES		BIT(0)
#define  INT_AESERR		BIT(1)
#define CRPT_AES_CTL		0x100
#define  AES_START		BIT(0)
#define  AES_STOP		BIT(1)
#define  AES_KEYSZ_128		(0 << 2)
#define  AES_KEYSZ_192		(1 << 2)
#define  AES_KEYSZ_256		(2 << 2)
#define  AES_DMALAST		BIT(5)
#define  AES_DMAEN		BIT(7)
#define  AES_MODE_ECB		(0 << 8)
#define  AES_MODE_CBC		(1 << 8)
#define  AES_MODE_CTR		(4 << 8)
#define  AES_ENCRYPT		BIT(16)
#define  AES_OUTSWAP		BIT(22)
#define  AES_INSWAP		BIT(23)
#define CRPT_AES_STS		0x104
#define  AES_STS_BUSY		BIT(0)
#define CRPT_AES0_KEY(n)	(0x110 + 4 * (n))
#define CRPT_AES0_IV(n)		(0x130 + 4 * (n))
#define CRPT_AES0_SADDR		0x140
#define CRPT_AES0_DADDR		0x144
#define CRPT_AES0_CNT		0x148

#define NUC970_DMA_BUFSZ	4096
#define NUC970_AES_TIMEOUT	msecs_to_jiffies(1000)

struct nuc970_crypto {
	struct device *dev;
	void __iomem *base;
	struct crypto_engine *engine;
	struct completion done;
	u32 intsts;
	u8 *inbuf;
	u8 *outbuf;
	dma_addr_t inbuf_dma;
	dma_addr_t outbuf_dma;
};

static struct nuc970_crypto *nuc970_crdev;

struct nuc970_aes_ctx {
	u32 key[AES_MAX_KEY_SIZE / 4];
	u32 keysz;
};

struct nuc970_aes_reqctx {
	u32 mode;
	bool encrypt;
};

struct nuc970_aes_alg {
	struct skcipher_engine_alg alg;
	u32 mode;
};

static int nuc970_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct nuc970_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	unsigned int i;

	switch (keylen) {
	case AES_KEYSIZE_128:
		ctx->keysz = AES_KEYSZ_128;
		break;
	case AES_KEYSIZE_192:
		ctx->keysz = AES_KEYSZ_192;
		break;
	case AES_KEYSIZE_256:
		ctx->keysz = AES_KEYSZ_256;
		break;
	default:
		return -EINVAL;
	}

	memset(ctx->key, 0, sizeof(ctx->key));
	for (i = 0; i < keylen / 4; i++)
		ctx->key[i] = get_unaligned_be32(key + 4 * i);
	return 0;
}

/* 128-bit big-endian counter add, matching the generic ctr template */
static void nuc970_ctr_add(u8 *ctr, u32 n)
{
	int i;

	for (i = AES_BLOCK_SIZE - 1; i >= 0 && n; i--) {
		u32 sum = ctr[i] + (n & 0xff);

		ctr[i] = sum;
		n = (n >> 8) + (sum >> 8);
	}
}

static int nuc970_aes_run_chunk(struct nuc970_crypto *cr, u32 ctl,
				const u8 *iv, unsigned int len)
{
	unsigned long left;
	int i;

	for (i = 0; i < 4; i++)
		writel(iv ? get_unaligned_be32(iv + 4 * i) : 0,
		       cr->base + CRPT_AES0_IV(i));

	writel(cr->inbuf_dma, cr->base + CRPT_AES0_SADDR);
	writel(cr->outbuf_dma, cr->base + CRPT_AES0_DADDR);
	writel(len, cr->base + CRPT_AES0_CNT);

	/*
	 * Same sequence as the BSP: idle the engine and clear stale flags
	 * (before re-arming the completion, so that a late interrupt from a
	 * timed-out run finds nothing pending), latch the configuration, then
	 * set START.
	 */
	writel(0, cr->base + CRPT_AES_CTL);
	writel(INT_AES | INT_AESERR, cr->base + CRPT_INTSTS);
	reinit_completion(&cr->done);
	writel(ctl, cr->base + CRPT_AES_CTL);
	writel(ctl | AES_DMALAST | AES_START, cr->base + CRPT_AES_CTL);

	left = wait_for_completion_timeout(&cr->done, NUC970_AES_TIMEOUT);
	if (!left) {
		writel(AES_STOP, cr->base + CRPT_AES_CTL);
		writel(INT_AES | INT_AESERR, cr->base + CRPT_INTSTS);
		dev_err(cr->dev, "AES DMA timeout\n");
		return -ETIMEDOUT;
	}
	if (cr->intsts & INT_AESERR) {
		dev_err(cr->dev, "AES engine error\n");
		return -EIO;
	}
	return 0;
}

static int nuc970_aes_do_one(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct nuc970_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct nuc970_aes_reqctx *rctx = skcipher_request_ctx(req);
	struct nuc970_crypto *cr = nuc970_crdev;
	u8 iv[AES_BLOCK_SIZE] = {};
	u8 next_iv[AES_BLOCK_SIZE];
	unsigned int done = 0, total = req->cryptlen;
	int err = 0, i;
	u32 ctl;

	if (rctx->mode != AES_MODE_ECB)
		memcpy(iv, req->iv, AES_BLOCK_SIZE);

	for (i = 0; i < AES_MAX_KEY_SIZE / 4; i++)
		writel(ctx->key[i], cr->base + CRPT_AES0_KEY(i));

	ctl = ctx->keysz | rctx->mode | AES_INSWAP | AES_OUTSWAP | AES_DMAEN;
	if (rctx->encrypt)
		ctl |= AES_ENCRYPT;

	while (done < total) {
		unsigned int len = min(total - done, (unsigned int)NUC970_DMA_BUFSZ);
		unsigned int dmalen, blocks;

		if (rctx->mode == AES_MODE_CTR) {
			/* never let the low 32 counter bits wrap inside a chunk */
			u32 lo = get_unaligned_be32(iv + 12);
			u64 room = ((u64)1 << 32) - lo;

			if ((u64)DIV_ROUND_UP(len, AES_BLOCK_SIZE) > room)
				len = room * AES_BLOCK_SIZE;
		}

		dmalen = round_up(len, AES_BLOCK_SIZE);
		blocks = dmalen / AES_BLOCK_SIZE;

		memset(cr->inbuf + len, 0, dmalen - len);
		scatterwalk_map_and_copy(cr->inbuf, req->src, done, len, 0);

		/* CBC decrypt: next IV is the last input ciphertext block */
		if (rctx->mode == AES_MODE_CBC && !rctx->encrypt)
			memcpy(next_iv, cr->inbuf + dmalen - AES_BLOCK_SIZE,
			       AES_BLOCK_SIZE);

		dma_sync_single_for_device(cr->dev, cr->inbuf_dma, dmalen,
					   DMA_TO_DEVICE);
		/* hand outbuf back to the device (no dirty lines may survive) */
		dma_sync_single_for_device(cr->dev, cr->outbuf_dma, dmalen,
					   DMA_BIDIRECTIONAL);
		err = nuc970_aes_run_chunk(cr, ctl,
					   rctx->mode == AES_MODE_ECB ? NULL : iv,
					   dmalen);
		dma_sync_single_for_cpu(cr->dev, cr->outbuf_dma, dmalen,
					DMA_BIDIRECTIONAL);
		if (err)
			break;

		scatterwalk_map_and_copy(cr->outbuf, req->dst, done, len, 1);

		if (rctx->mode == AES_MODE_CBC) {
			if (rctx->encrypt)
				memcpy(iv, cr->outbuf + dmalen - AES_BLOCK_SIZE,
				       AES_BLOCK_SIZE);
			else
				memcpy(iv, next_iv, AES_BLOCK_SIZE);
		} else if (rctx->mode == AES_MODE_CTR) {
			nuc970_ctr_add(iv, blocks);
		}
		done += len;
	}

	if (!err && rctx->mode != AES_MODE_ECB)
		memcpy(req->iv, iv, AES_BLOCK_SIZE);

	/*
	 * Scrub the bounce buffers and write the zeroes back to RAM right
	 * away (ARM926 D-cache is write-back), so no plaintext/key stream is
	 * left behind and no dirty line can later be evicted on top of data
	 * the engine is writing.
	 */
	memzero_explicit(cr->inbuf, NUC970_DMA_BUFSZ);
	memzero_explicit(cr->outbuf, NUC970_DMA_BUFSZ);
	dma_sync_single_for_device(cr->dev, cr->inbuf_dma, NUC970_DMA_BUFSZ,
				   DMA_TO_DEVICE);
	dma_sync_single_for_device(cr->dev, cr->outbuf_dma, NUC970_DMA_BUFSZ,
				   DMA_BIDIRECTIONAL);

	local_bh_disable();
	crypto_finalize_skcipher_request(engine, req, err);
	local_bh_enable();
	return 0;
}

static int nuc970_aes_crypt(struct skcipher_request *req, bool encrypt)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct skcipher_alg *salg = crypto_skcipher_alg(tfm);
	struct nuc970_aes_alg *alg = container_of(salg, struct nuc970_aes_alg, alg.base);
	struct nuc970_aes_reqctx *rctx = skcipher_request_ctx(req);

	if (alg->mode != AES_MODE_CTR && !IS_ALIGNED(req->cryptlen, AES_BLOCK_SIZE))
		return -EINVAL;
	if (!req->cryptlen)
		return 0;

	rctx->mode = alg->mode;
	rctx->encrypt = encrypt;
	return crypto_transfer_skcipher_request_to_engine(nuc970_crdev->engine, req);
}

static int nuc970_aes_encrypt(struct skcipher_request *req)
{
	return nuc970_aes_crypt(req, true);
}

static int nuc970_aes_decrypt(struct skcipher_request *req)
{
	return nuc970_aes_crypt(req, false);
}

static int nuc970_aes_init_tfm(struct crypto_skcipher *tfm)
{
	crypto_skcipher_set_reqsize(tfm, sizeof(struct nuc970_aes_reqctx));
	return 0;
}

#define NUC970_AES_ALG(_name, _drv, _mode, _bs, _iv)			\
{									\
	.mode = _mode,							\
	.alg.base = {							\
		.base.cra_name		= _name,			\
		.base.cra_driver_name	= _drv,				\
		.base.cra_priority	= 300,				\
		.base.cra_flags		= CRYPTO_ALG_ASYNC |		\
					  CRYPTO_ALG_KERN_DRIVER_ONLY,	\
		.base.cra_blocksize	= _bs,				\
		.base.cra_ctxsize	= sizeof(struct nuc970_aes_ctx),\
		.base.cra_module	= THIS_MODULE,			\
		.min_keysize		= AES_MIN_KEY_SIZE,		\
		.max_keysize		= AES_MAX_KEY_SIZE,		\
		.ivsize			= _iv,				\
		.chunksize		= AES_BLOCK_SIZE,		\
		.setkey			= nuc970_aes_setkey,		\
		.encrypt		= nuc970_aes_encrypt,		\
		.decrypt		= nuc970_aes_decrypt,		\
		.init			= nuc970_aes_init_tfm,		\
	},								\
	.alg.op.do_one_request = nuc970_aes_do_one,			\
}

static struct nuc970_aes_alg nuc970_aes_algs[] = {
	NUC970_AES_ALG("ecb(aes)", "ecb-aes-nuc970", AES_MODE_ECB, AES_BLOCK_SIZE, 0),
	NUC970_AES_ALG("cbc(aes)", "cbc-aes-nuc970", AES_MODE_CBC, AES_BLOCK_SIZE,
		       AES_BLOCK_SIZE),
	NUC970_AES_ALG("ctr(aes)", "ctr-aes-nuc970", AES_MODE_CTR, 1, AES_BLOCK_SIZE),
};

static irqreturn_t nuc970_crypto_irq(int irq, void *data)
{
	struct nuc970_crypto *cr = data;
	u32 sts = readl(cr->base + CRPT_INTSTS) & (INT_AES | INT_AESERR);

	if (!sts)
		return IRQ_NONE;

	writel(sts, cr->base + CRPT_INTSTS);
	cr->intsts = sts;
	complete(&cr->done);
	return IRQ_HANDLED;
}

static void nuc970_crypto_unregister(void *data)
{
	int i;

	for (i = ARRAY_SIZE(nuc970_aes_algs) - 1; i >= 0; i--)
		crypto_engine_unregister_skcipher(&nuc970_aes_algs[i].alg);
	nuc970_crdev = NULL;
}

static void nuc970_crypto_engine_exit(void *data)
{
	crypto_engine_exit(data);
}

static void nuc970_crypto_unmap(void *data)
{
	struct nuc970_crypto *cr = data;

	dma_unmap_single(cr->dev, cr->inbuf_dma, NUC970_DMA_BUFSZ, DMA_TO_DEVICE);
	dma_unmap_single(cr->dev, cr->outbuf_dma, NUC970_DMA_BUFSZ, DMA_BIDIRECTIONAL);
}

static int nuc970_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_crypto *cr;
	struct clk *clk;
	int irq, err, i;

	if (nuc970_crdev)
		return -EBUSY;

	cr = devm_kzalloc(dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;
	cr->dev = dev;
	init_completion(&cr->done);

	cr->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cr->base))
		return PTR_ERR(cr->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	cr->inbuf = devm_kzalloc(dev, NUC970_DMA_BUFSZ, GFP_KERNEL);
	cr->outbuf = devm_kzalloc(dev, NUC970_DMA_BUFSZ, GFP_KERNEL);
	if (!cr->inbuf || !cr->outbuf)
		return -ENOMEM;

	cr->inbuf_dma = dma_map_single(dev, cr->inbuf, NUC970_DMA_BUFSZ, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, cr->inbuf_dma))
		return -ENOMEM;
	/*
	 * Bidirectional so that the CPU-side scrub of outbuf can be cleaned
	 * to RAM (a FROM_DEVICE sync only invalidates and would drop it).
	 */
	cr->outbuf_dma = dma_map_single(dev, cr->outbuf, NUC970_DMA_BUFSZ,
					DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, cr->outbuf_dma)) {
		dma_unmap_single(dev, cr->inbuf_dma, NUC970_DMA_BUFSZ, DMA_TO_DEVICE);
		return -ENOMEM;
	}
	err = devm_add_action_or_reset(dev, nuc970_crypto_unmap, cr);
	if (err)
		return err;

	writel(0, cr->base + CRPT_AES_CTL);
	writel(INT_AES | INT_AESERR, cr->base + CRPT_INTSTS);
	writel(INT_AES | INT_AESERR, cr->base + CRPT_INTEN);

	err = devm_request_irq(dev, irq, nuc970_crypto_irq, 0, dev_name(dev), cr);
	if (err)
		return err;

	cr->engine = crypto_engine_alloc_init(dev, true);
	if (!cr->engine)
		return -ENOMEM;
	err = devm_add_action_or_reset(dev, nuc970_crypto_engine_exit, cr->engine);
	if (err)
		return err;
	err = crypto_engine_start(cr->engine);
	if (err)
		return err;

	nuc970_crdev = cr;
	platform_set_drvdata(pdev, cr);

	for (i = 0; i < ARRAY_SIZE(nuc970_aes_algs); i++) {
		err = crypto_engine_register_skcipher(&nuc970_aes_algs[i].alg);
		if (err) {
			dev_err(dev, "failed to register %s: %d\n",
				nuc970_aes_algs[i].alg.base.base.cra_driver_name, err);
			while (--i >= 0)
				crypto_engine_unregister_skcipher(&nuc970_aes_algs[i].alg);
			nuc970_crdev = NULL;
			return err;
		}
	}
	err = devm_add_action_or_reset(dev, nuc970_crypto_unregister, NULL);
	if (err)
		return err;

	dev_info(dev, "NUC970 AES engine registered\n");
	return 0;
}

static const struct of_device_id nuc970_crypto_of_match[] = {
	{ .compatible = "nuvoton,nuc970-crypto" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_crypto_of_match);

static struct platform_driver nuc970_crypto_driver = {
	.probe	= nuc970_crypto_probe,
	.driver	= {
		.name		= "nuc970-crypto",
		.of_match_table	= nuc970_crypto_of_match,
	},
};
module_platform_driver(nuc970_crypto_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 AES accelerator driver");
MODULE_LICENSE("GPL");
