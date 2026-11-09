// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 MTP key store: read-only status for AF_ALG.
 *
 * The vendor kernel exposed the on-chip MTP (multi-time programmable) AES
 * key store through a "mtp" skcipher and private AF_ALG socket options.
 * Its userland (libmtp_api.so) asks for the status with
 *
 *	socket(AF_ALG, SOCK_SEQPACKET, 0);
 *	bind(fd, { .salg_type = "skcipher", .salg_name = "mtp" });
 *	status = setsockopt(fd, SOL_ALG, 205, NULL, 0);
 *
 * and expects the MTP_STATUS register as the return value, or 0xffff when
 * the block cannot be enabled. Only that query is implemented here: key
 * programming and locking burn the OTP irreversibly and are deliberately
 * left out, so the only key length this algorithm accepts is the 1-byte
 * status request issued by crypto/af_alg.c.
 */

#include <crypto/internal/skcipher.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define MTP_KEYEN		0x00
#define  MTP_KEYEN_KEYEN	BIT(0)
#define MTP_STATUS		0x40
#define  MTP_STATUS_MTPEN	BIT(0)
#define  MTP_STATUS_KEYVALID	BIT(1)
#define  MTP_STATUS_NONPRG	BIT(2)
#define  MTP_STATUS_BUSY	BIT(24)
#define MTP_REGLCTL		0x50

#define MTP_STATUS_FAILED	0xffff	/* vendor ABI */

struct nuc970_mtp {
	void __iomem *base;
	struct device *dev;
	struct mutex lock;
};

static struct nuc970_mtp *nuc970_mtp;

static int nuc970_mtp_status(struct nuc970_mtp *mtp)
{
	u32 st;
	int err;

	mutex_lock(&mtp->lock);

	/* unlock the MTP registers, enable the key store, relock */
	writel(0x59, mtp->base + MTP_REGLCTL);
	writel(0x16, mtp->base + MTP_REGLCTL);
	writel(0x88, mtp->base + MTP_REGLCTL);
	writel(readl(mtp->base + MTP_KEYEN) | MTP_KEYEN_KEYEN,
	       mtp->base + MTP_KEYEN);
	writel(0, mtp->base + MTP_REGLCTL);

	err = readl_poll_timeout(mtp->base + MTP_STATUS, st,
				 (st & MTP_STATUS_MTPEN) &&
				 !(st & MTP_STATUS_BUSY) &&
				 (st & (MTP_STATUS_NONPRG | MTP_STATUS_KEYVALID)),
				 10, 100000);

	mutex_unlock(&mtp->lock);

	if (err) {
		dev_warn(mtp->dev, "MTP enable failed, status %#x\n", st);
		return MTP_STATUS_FAILED;
	}
	return st;
}

/*
 * The vendor ABI returns the status through setkey. af_alg passes a 1-byte
 * key for ALG_MTP_STATUS; min/max_keysize make every other length (i.e. a
 * key to program) fail with -EINVAL before this is called.
 */
static int nuc970_mtp_setkey(struct crypto_skcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	if (!nuc970_mtp)
		return -ENODEV;
	return nuc970_mtp_status(nuc970_mtp);
}

static int nuc970_mtp_crypt(struct skcipher_request *req)
{
	return -EOPNOTSUPP;
}

static struct skcipher_alg nuc970_mtp_alg = {
	.base.cra_name		= "mtp",
	.base.cra_driver_name	= "nuc970-mtp",
	.base.cra_priority	= 300,
	.base.cra_blocksize	= 1,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= 1,
	.max_keysize		= 1,
	.setkey			= nuc970_mtp_setkey,
	.encrypt		= nuc970_mtp_crypt,
	.decrypt		= nuc970_mtp_crypt,
};

static int nuc970_mtp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuc970_mtp *mtp;
	struct clk *clk;
	int err;

	mtp = devm_kzalloc(dev, sizeof(*mtp), GFP_KERNEL);
	if (!mtp)
		return -ENOMEM;
	mtp->dev = dev;
	mutex_init(&mtp->lock);

	mtp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mtp->base))
		return PTR_ERR(mtp->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no clock\n");

	nuc970_mtp = mtp;
	err = crypto_register_skcipher(&nuc970_mtp_alg);
	if (err) {
		nuc970_mtp = NULL;
		return dev_err_probe(dev, err, "cannot register mtp\n");
	}

	dev_info(dev, "MTP key store, status %#x (read-only)\n",
		 readl(mtp->base + MTP_STATUS));
	return 0;
}

static void nuc970_mtp_remove(struct platform_device *pdev)
{
	crypto_unregister_skcipher(&nuc970_mtp_alg);
	nuc970_mtp = NULL;
}

static const struct of_device_id nuc970_mtp_of_match[] = {
	{ .compatible = "nuvoton,nuc970-mtp" },
	{ }
};
MODULE_DEVICE_TABLE(of, nuc970_mtp_of_match);

static struct platform_driver nuc970_mtp_driver = {
	.probe	= nuc970_mtp_probe,
	.remove	= nuc970_mtp_remove,
	.driver	= {
		.name		= "nuc970-mtp",
		.of_match_table	= nuc970_mtp_of_match,
	},
};
module_platform_driver(nuc970_mtp_driver);

MODULE_DESCRIPTION("Nuvoton NUC970 MTP key store status (read-only)");
MODULE_LICENSE("GPL");
