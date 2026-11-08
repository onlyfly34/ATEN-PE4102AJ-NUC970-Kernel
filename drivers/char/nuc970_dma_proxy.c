// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * /dev/dma_proxy: user-space access to a Nuvoton NUC970 GDMA memcpy channel
 *
 * Port of the char-device DMA proxy (nuc970_dma_proxy.[ch]) from
 * Nuvoton's NUC970 Linux 3.10 BSP (Copyright (c) 2015 Nuvoton technology
 * corporation) to the current dmaengine API. The device is instantiated by
 * the GDMA driver (drivers/dma/nuc970-gdma.c) as platform device
 * "nuc970-dma-proxy" once its channels are registered.
 *
 * User ABI, kept identical to the 3.10 BSP:
 *
 *  - node /dev/dma_proxy (dynamic major, class "nuc970_dma");
 *
 *  - mmap(len) allocates a fresh write-combined, physically contiguous
 *    buffer of exactly @len bytes and maps it. It becomes the "current"
 *    buffer for everybody (the BSP kept it in globals); the previous
 *    current buffer is released once it is no longer mapped (the BSP
 *    leaked every buffer instead). An offset (vm_pgoff) is rejected, as it
 *    was by the BSP's dma_mmap_writecombine();
 *
 *  - ioctl(): the BSP handler did "cmd = *(unsigned int *)command", i.e. it
 *    took the ioctl request number as a POINTER to the real command word
 *    (that only works on ARMv5 because the kernel could dereference user
 *    addresses directly). That pointer form is what works here too: the
 *    request is read with get_user() as a pointer to a u32 holding 1/2.
 *    A plain request number 1 is also accepted, but a plain 2 can never
 *    reach this driver: the VFS consumes it as FIGETBSZ (_IO(0, 2)) for
 *    every file type, as 3.10 did. Unknown commands return 0, as in the
 *    BSP;
 *
 *  - NUC970_DMA_ALLOC (1), arg struct nuc970_mem_alloc (16 bytes): returns
 *    the current buffer's bus address in phy_addr and its KERNEL virtual
 *    address in vir_addr (0/0 before the first mmap); size and kvir_addr
 *    are passed back unchanged;
 *
 *  - NCU970_DMA_TRANSFER (2, BSP spelling), arg struct nuc970_mem_dma_param
 *    (16 bytes): first writes the test ramp 0,1,2,... over the first @size
 *    bytes of the current buffer (BSP behaviour; bounded to the buffer here
 *    and switchable with the test_pattern module parameter), then copies
 *    @size bytes from bus address src_addr to dst_addr with the GDMA and
 *    returns when the copy has finished. @cfg was the 3.10 dmaengine
 *    unmap-control flags word and is ignored.
 *
 * Deviations, all on paths that used to oops or corrupt memory in the BSP:
 * size <= 0 gives -EINVAL, an unreadable command pointer gives -ENOTTY, a
 * hung or failed transfer gives -ETIMEDOUT/-EIO, and because TRANSFER can
 * DMA to any physical address it needs CAP_SYS_RAWIO (like /dev/mem).
 */

#include <linux/capability.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define DMA_PROXY_NAME		"dma_proxy"
#define DMA_PROXY_CLASS		"nuc970_dma"

/* ioctl command words, from the BSP's nuc970_dma_proxy.h */
#define NUC970_DMA_ALLOC	1
#define NCU970_DMA_TRANSFER	2

/* BSP layouts (built with USE_DMA_ALLOC=1): 4 x 32 bit, 16 bytes each */
struct nuc970_mem_alloc {
	__s32 size;
	__u32 vir_addr;
	__u32 phy_addr;
	__u32 kvir_addr;
};

struct nuc970_mem_dma_param {
	__s32 size;
	__u32 src_addr;
	__u32 dst_addr;
	__s32 cfg;
};

static bool test_pattern = true;
module_param(test_pattern, bool, 0644);
MODULE_PARM_DESC(test_pattern,
		 "fill the mmap()ed buffer with 0,1,2,... before each transfer, like the 3.10 BSP (default: on)");

struct dma_proxy_buf {
	struct kref ref;
	struct device *dev;
	size_t size;
	void *cpu;
	dma_addr_t dma;
};

struct dma_proxy {
	struct dma_chan *chan;
	struct device *dma_dev;		/* the GDMA, owner of the buffers */
	dev_t devt;
	struct cdev cdev;
	struct device *node;
	spinlock_t buf_lock;		/* protects cur */
	struct dma_proxy_buf *cur;
	struct mutex xfer_lock;		/* one transfer at a time */
};

struct dma_proxy_done {
	struct completion done;
	enum dmaengine_tx_result result;
};

static const struct class dma_proxy_class = {
	.name = DMA_PROXY_CLASS,
};

static void dma_proxy_buf_release(struct kref *ref)
{
	struct dma_proxy_buf *buf = container_of(ref, struct dma_proxy_buf, ref);

	dma_free_wc(buf->dev, buf->size, buf->cpu, buf->dma);
	put_device(buf->dev);
	kfree(buf);
}

static void dma_proxy_buf_put(struct dma_proxy_buf *buf)
{
	if (buf)
		kref_put(&buf->ref, dma_proxy_buf_release);
}

static struct dma_proxy_buf *dma_proxy_get_cur(struct dma_proxy *p)
{
	struct dma_proxy_buf *buf;

	spin_lock(&p->buf_lock);
	buf = p->cur;
	if (buf)
		kref_get(&buf->ref);
	spin_unlock(&p->buf_lock);

	return buf;
}

static void dma_proxy_vm_open(struct vm_area_struct *vma)
{
	struct dma_proxy_buf *buf = vma->vm_private_data;

	kref_get(&buf->ref);
}

static void dma_proxy_vm_close(struct vm_area_struct *vma)
{
	dma_proxy_buf_put(vma->vm_private_data);
}

static const struct vm_operations_struct dma_proxy_vm_ops = {
	.open = dma_proxy_vm_open,
	.close = dma_proxy_vm_close,
};

static int dma_proxy_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dma_proxy *p = file->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	struct dma_proxy_buf *buf, *old;
	int ret;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	kref_init(&buf->ref);			/* reference held by the vma */
	buf->dev = get_device(p->dma_dev);
	buf->size = size;
	buf->cpu = dma_alloc_wc(buf->dev, size, &buf->dma, GFP_KERNEL);
	if (!buf->cpu) {
		put_device(buf->dev);
		kfree(buf);
		return -ENOMEM;
	}

	ret = dma_mmap_wc(buf->dev, vma, buf->cpu, buf->dma, size);
	if (ret) {
		dma_proxy_buf_put(buf);
		return ret;
	}

	vma->vm_private_data = buf;
	vma->vm_ops = &dma_proxy_vm_ops;

	kref_get(&buf->ref);			/* reference held as "current" */
	spin_lock(&p->buf_lock);
	old = p->cur;
	p->cur = buf;
	spin_unlock(&p->buf_lock);
	dma_proxy_buf_put(old);

	return 0;
}

static long dma_proxy_alloc(struct dma_proxy *p, void __user *uarg)
{
	struct nuc970_mem_alloc param;
	struct dma_proxy_buf *buf;

	if (copy_from_user(&param, uarg, sizeof(param)))
		return -EFAULT;

	buf = dma_proxy_get_cur(p);
	if (buf) {
		param.phy_addr = (u32)buf->dma;
		/* the BSP handed out the kernel mapping, keep doing so */
		param.vir_addr = (u32)(uintptr_t)buf->cpu;
		dma_proxy_buf_put(buf);
	} else {
		param.phy_addr = 0;
		param.vir_addr = 0;
	}

	if (copy_to_user(uarg, &param, sizeof(param)))
		return -EFAULT;

	return 0;
}

static void dma_proxy_callback(void *arg, const struct dmaengine_result *res)
{
	struct dma_proxy_done *d = arg;

	d->result = res ? res->result : DMA_TRANS_NOERROR;
	complete(&d->done);
}

static long dma_proxy_transfer(struct dma_proxy *p, void __user *uarg)
{
	struct nuc970_mem_dma_param param;
	struct dma_async_tx_descriptor *tx;
	struct dma_proxy_buf *buf;
	struct dma_proxy_done d;
	unsigned long timeout;
	dma_cookie_t cookie;
	long ret = 0;

	if (copy_from_user(&param, uarg, sizeof(param)))
		return -EFAULT;

	if (param.size <= 0)
		return -EINVAL;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	mutex_lock(&p->xfer_lock);

	buf = dma_proxy_get_cur(p);
	if (buf && test_pattern) {
		size_t n = min_t(size_t, param.size, buf->size);
		u8 *v = buf->cpu;
		size_t i;

		for (i = 0; i < n; i++)
			v[i] = i;
	}

	tx = dmaengine_prep_dma_memcpy(p->chan, param.dst_addr, param.src_addr,
				       param.size,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		ret = -ENOMEM;
		goto out;
	}

	init_completion(&d.done);
	d.result = DMA_TRANS_NOERROR;
	tx->callback_result = dma_proxy_callback;
	tx->callback_param = &d;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret)
		goto out;

	dma_async_issue_pending(p->chan);

	/* generous: 1 s plus 1 ms per 16 KiB */
	timeout = msecs_to_jiffies(1000 + (param.size >> 14));
	if (!wait_for_completion_timeout(&d.done, timeout)) {
		dmaengine_terminate_sync(p->chan);
		ret = -ETIMEDOUT;
	} else if (d.result != DMA_TRANS_NOERROR) {
		ret = -EIO;
	}

out:
	dma_proxy_buf_put(buf);
	mutex_unlock(&p->xfer_lock);
	return ret;
}

static long dma_proxy_ioctl(struct file *file, unsigned int command,
			    unsigned long arg)
{
	struct dma_proxy *p = file->private_data;
	void __user *uarg = (void __user *)arg;
	u32 cmd;

	/* a plain 2 is FIGETBSZ and never gets here, see the header comment */
	if (command == NUC970_DMA_ALLOC || command == NCU970_DMA_TRANSFER)
		cmd = command;
	else if (get_user(cmd, (u32 __user *)(uintptr_t)command))
		return -ENOTTY;		/* BSP ABI: request is a pointer */

	switch (cmd) {
	case NUC970_DMA_ALLOC:
		return dma_proxy_alloc(p, uarg);
	case NCU970_DMA_TRANSFER:
		return dma_proxy_transfer(p, uarg);
	default:
		return 0;		/* as the BSP */
	}
}

static int dma_proxy_open(struct inode *inode, struct file *file)
{
	file->private_data = container_of(inode->i_cdev, struct dma_proxy, cdev);
	return 0;
}

static const struct file_operations dma_proxy_fops = {
	.owner = THIS_MODULE,
	.open = dma_proxy_open,
	.mmap = dma_proxy_mmap,
	.unlocked_ioctl = dma_proxy_ioctl,
};

static int dma_proxy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_proxy *p;
	dma_cap_mask_t mask;
	int ret;

	BUILD_BUG_ON(sizeof(struct nuc970_mem_alloc) != 16);
	BUILD_BUG_ON(sizeof(struct nuc970_mem_dma_param) != 16);

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	spin_lock_init(&p->buf_lock);
	mutex_init(&p->xfer_lock);

	/* Like the BSP: hold one memcpy channel for the lifetime of the node */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	p->chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(p->chan))
		return dev_err_probe(dev, PTR_ERR(p->chan),
				     "cannot get a memcpy DMA channel\n");
	p->dma_dev = p->chan->device->dev;

	ret = alloc_chrdev_region(&p->devt, 0, 1, DMA_PROXY_NAME);
	if (ret) {
		dev_err(dev, "cannot allocate a char device number\n");
		goto err_chan;
	}

	cdev_init(&p->cdev, &dma_proxy_fops);
	p->cdev.owner = THIS_MODULE;
	ret = cdev_add(&p->cdev, p->devt, 1);
	if (ret) {
		dev_err(dev, "cannot add char device\n");
		goto err_region;
	}

	p->node = device_create(&dma_proxy_class, dev, p->devt, p,
				DMA_PROXY_NAME);
	if (IS_ERR(p->node)) {
		ret = PTR_ERR(p->node);
		dev_err(dev, "cannot create /dev/%s\n", DMA_PROXY_NAME);
		goto err_cdev;
	}

	platform_set_drvdata(pdev, p);
	dev_info(dev, "/dev/%s on %s\n", DMA_PROXY_NAME, dma_chan_name(p->chan));
	return 0;

err_cdev:
	cdev_del(&p->cdev);
err_region:
	unregister_chrdev_region(p->devt, 1);
err_chan:
	dma_release_channel(p->chan);
	return ret;
}

static void dma_proxy_remove(struct platform_device *pdev)
{
	struct dma_proxy *p = platform_get_drvdata(pdev);
	struct dma_proxy_buf *buf;

	device_destroy(&dma_proxy_class, p->devt);
	cdev_del(&p->cdev);
	unregister_chrdev_region(p->devt, 1);

	spin_lock(&p->buf_lock);
	buf = p->cur;
	p->cur = NULL;
	spin_unlock(&p->buf_lock);
	/* may free the buffer: dma_free_wc() can sleep, so not under the lock */
	dma_proxy_buf_put(buf);

	dmaengine_terminate_sync(p->chan);
	dma_release_channel(p->chan);
}

static struct platform_driver dma_proxy_driver = {
	.probe = dma_proxy_probe,
	.remove = dma_proxy_remove,
	.driver = {
		.name = "nuc970-dma-proxy",
		/* open files reference the driver state */
		.suppress_bind_attrs = true,
	},
};

static int __init dma_proxy_init(void)
{
	int ret;

	ret = class_register(&dma_proxy_class);
	if (ret)
		return ret;

	ret = platform_driver_register(&dma_proxy_driver);
	if (ret)
		class_unregister(&dma_proxy_class);

	return ret;
}
module_init(dma_proxy_init);

static void __exit dma_proxy_exit(void)
{
	platform_driver_unregister(&dma_proxy_driver);
	class_unregister(&dma_proxy_class);
}
module_exit(dma_proxy_exit);

MODULE_ALIAS("platform:nuc970-dma-proxy");
MODULE_DESCRIPTION("Nuvoton NUC970 GDMA /dev/dma_proxy (3.10 BSP ABI)");
MODULE_LICENSE("GPL");
