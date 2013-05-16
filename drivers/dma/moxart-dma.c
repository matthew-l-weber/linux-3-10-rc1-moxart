/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/cacheflush.h>

#include "dmaengine.h"
#include "virt-dma.h"
#include "moxart-dma.h"

/*	moxart_dma_reg_cfg:s dest_req_no and source_req_no seem to be hardcoded,
	for example MMC must always request channels where dma_slave_config:s
	slave_id is 5, the known numbers are:

	#define APB_DMA_SPI_TX_REQ_NO       1
	#define APB_DMA_SPI_RX_REQ_NO       2
	#define APB_DMA_SD_REQ_NO			5
	#define APB_DMA_AC97_TX_REQ_NO      6
	#define APB_DMA_AC97_RX_REQ_NO      7
	#define APB_DMA_USB_DEVICE_REQ_NO   9
*/

static DEFINE_SPINLOCK(dma_lock);

struct moxart_dma_chan {
	struct virt_dma_chan	vchan;
	int						ch_num;
	bool					alloced;
	int						error_flag;
	struct moxart_dma_reg	*reg;
	void					(*callback)(void *param);
	void					*callback_param;
	struct completion		dma_complete;
	struct dma_slave_config	cfg;
	struct dma_async_tx_descriptor tx_desc;
};

struct moxart_dma_container {
	int						ctlr;
	struct dma_device		dma_slave;
	struct moxart_dma_chan	slave_chans[APB_DMA_MAX_CHANNEL];
};

struct moxart_dma_container *mdc;

static inline struct moxart_dma_container *to_moxart_dma_container
	(struct dma_device *d)
{
	return container_of(d, struct moxart_dma_container, dma_slave);
}

static inline struct moxart_dma_chan *to_moxart_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct moxart_dma_chan, vchan.chan);
}

static int moxart_terminate_all(struct dma_chan *chan)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	union moxart_dma_reg_cfg mcfg;
	unsigned long flags;

	pr_debug("%s: mchan=%p\n", __func__, mchan);

	spin_lock_irqsave(&dma_lock, flags);

	mcfg.ul = readl(&mchan->reg->cfg.ul);
	mcfg.ul &= ~(APB_DMA_ENABLE | APB_DMA_FIN_INT_EN | APB_DMA_ERR_INT_EN);
	writel(mcfg.ul, &mchan->reg->cfg.ul);

	spin_unlock_irqrestore(&dma_lock, flags);

	return 0;
}

static int moxart_slave_config(struct dma_chan *chan,
	struct dma_slave_config *cfg)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	union moxart_dma_reg_cfg mcfg;
	unsigned long flags;
	unsigned int data_width, data_inc;

	spin_lock_irqsave(&dma_lock, flags);

	memcpy(&mchan->cfg, cfg, sizeof(mchan->cfg));

	mcfg.ul = readl(&mchan->reg->cfg.ul);
	mcfg.bits.burst = APB_DMAB_BURST_MODE;

	switch (mchan->cfg.src_addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		data_width = APB_DMAB_DATA_WIDTH_1;
		data_inc = APB_DMAB_DEST_INC_1_4;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		data_width = APB_DMAB_DATA_WIDTH_2;
		data_inc = APB_DMAB_DEST_INC_2_8;
		break;
	default:
		data_width = APB_DMAB_DATA_WIDTH_4;
		data_inc = APB_DMAB_DEST_INC_4_16;
		break;
	}

	if (mchan->cfg.direction == DMA_MEM_TO_DEV) {
		mcfg.bits.data_width = data_width;
		mcfg.bits.dest_sel = APB_DMAB_DEST_APB;
		mcfg.bits.dest_inc = APB_DMAB_DEST_INC_0;
		mcfg.bits.source_sel = APB_DMAB_SOURCE_AHB;
		mcfg.bits.source_inc = data_inc;

		mcfg.bits.dest_req_no = mchan->cfg.slave_id;
		mcfg.bits.source_req_no = 0;
	} else {
		mcfg.bits.data_width = data_width;
		mcfg.bits.dest_sel = APB_DMAB_SOURCE_AHB;
		mcfg.bits.dest_inc = data_inc;
		mcfg.bits.source_sel = APB_DMAB_DEST_APB;
		mcfg.bits.source_inc = APB_DMAB_DEST_INC_0;

		mcfg.bits.dest_req_no = 0;
		mcfg.bits.source_req_no = mchan->cfg.slave_id;
	}

	writel(mcfg.ul, &mchan->reg->cfg.ul);

	spin_unlock_irqrestore(&dma_lock, flags);

	return 0;
}

static int moxart_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	int ret = 0;
	struct dma_slave_config *config;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		moxart_terminate_all(chan);
		break;
	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;
		ret = moxart_slave_config(chan, config);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static dma_cookie_t moxart_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(tx->chan);
	dma_cookie_t cookie;
	union moxart_dma_reg_cfg mcfg;
	unsigned long flags;

	mchan->callback = tx->callback;
	mchan->callback_param = tx->callback_param;
	mchan->error_flag = 0;

	pr_debug("%s: mchan=%p mchan->ch_num=%d mchan->reg=%p\n",
		__func__, mchan, mchan->ch_num, mchan->reg);

	spin_lock_irqsave(&dma_lock, flags);

	cookie = dma_cookie_assign(tx);

	mcfg.ul = readl(&mchan->reg->cfg.ul);
	mcfg.ul |= (APB_DMA_FIN_INT_EN | APB_DMA_ERR_INT_EN);
	writel(mcfg.ul, &mchan->reg->cfg.ul);

	spin_unlock_irqrestore(&dma_lock, flags);

	return cookie;
}

static struct dma_async_tx_descriptor *moxart_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long tx_flags, void *context)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	unsigned long flags;
	union moxart_dma_reg_cfg mcfg;
	unsigned int size, adr_width;

	spin_lock_irqsave(&dma_lock, flags);

	if (direction == DMA_MEM_TO_DEV) {
		writel(virt_to_phys((void *)sg_dma_address(&sgl[0])),
			&mchan->reg->source_addr);
		writel(mchan->cfg.dst_addr, &mchan->reg->dest_addr);
		adr_width = mchan->cfg.src_addr_width;
	} else {
		writel(mchan->cfg.src_addr, &mchan->reg->source_addr);
		writel(virt_to_phys((void *)sg_dma_address(&sgl[0])),
			&mchan->reg->dest_addr);
		adr_width = mchan->cfg.dst_addr_width;
	}

	size = sgl->length >> adr_width;
	writel(size, &mchan->reg->cycles);
	pr_debug("%s: set %d DMA cycles (sgl->length=%d adr_width=%d)\n",
		__func__, size, sgl->length, adr_width);
	/*	size == 4 on 64 bytes copied, i.e. once cycle copies 16 bytes
		(when data_width == APB_DMAB_DATA_WIDTH_4) */
	pr_debug("%s: mcfg.ul=%x read from &mchan->reg->cfg.ul=%x\n",
		__func__, mcfg.ul, (unsigned int)&mchan->reg->cfg.ul);

	dma_async_tx_descriptor_init(&mchan->tx_desc, chan);
	mchan->tx_desc.tx_submit = moxart_tx_submit;

	spin_unlock_irqrestore(&dma_lock, flags);

	return &mchan->tx_desc;
}

static struct platform_driver moxart_driver;

bool moxart_filter_fn(struct dma_chan *chan, void *param)
{
	if (chan->device->dev->driver == &moxart_driver.driver) {
		struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
		unsigned int ch_req = *(unsigned int *)param;
		pr_debug("%s: mchan=%p ch_req=%d mchan->ch_num=%d\n",
			__func__, mchan, ch_req, mchan->ch_num);
		return ch_req == mchan->ch_num;
	} else {
		pr_debug("%s: device not registered to this DMA engine\n",
			__func__);
		return false;
	}
}
EXPORT_SYMBOL(moxart_filter_fn);

static int moxart_alloc_chan_resources(struct dma_chan *chan)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	int i;
	bool found = false;

	for (i = 0; i < APB_DMA_MAX_CHANNEL; i++) {
		if (i == mchan->ch_num
			&& !mchan->alloced) {
			pr_debug("%s: non allocated channel=%d found, allocating!\n",
				__func__, mchan->ch_num);
			mchan->alloced = true;
			found = true;
			break;
		}
	}

	if (!found)
		return -ENODEV;

	return 0;
}

static void moxart_free_chan_resources(struct dma_chan *chan)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	struct device *dev = chan->device->dev;

	mchan->alloced = false;
	dev_info(dev, "freeing channel for %u\n", mchan->ch_num);
}

/* Send pending descriptor to hardware */
static void moxart_issue_pending(struct dma_chan *chan)
{
	struct moxart_dma_chan *mchan = to_moxart_dma_chan(chan);
	union moxart_dma_reg_cfg mcfg;
	unsigned long flags;

	pr_debug("%s: mchan=%p\n", __func__, mchan);

	spin_lock_irqsave(&dma_lock, flags);

	mcfg.ul = readl(&mchan->reg->cfg.ul);
	mcfg.ul |= APB_DMA_ENABLE;
	writel(mcfg.ul, &mchan->reg->cfg.ul);

	spin_unlock_irqrestore(&dma_lock, flags);
}

/* Check request completion status */
static enum dma_status moxart_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_SUCCESS || !txstate)
		return ret;

	return ret;
}

static void moxart_dma_init(struct dma_device *dma, struct device *dev)
{
	dma->device_prep_slave_sg = moxart_prep_slave_sg;
	dma->device_alloc_chan_resources = moxart_alloc_chan_resources;
	dma->device_free_chan_resources = moxart_free_chan_resources;
	dma->device_issue_pending = moxart_issue_pending;
	dma->device_tx_status = moxart_tx_status;
	dma->device_control = moxart_control;
	dma->dev = dev;

	INIT_LIST_HEAD(&dma->channels);
}

static void __init moxart_dma_chan_init(void __iomem *dma_base_addr,
	struct dma_device *dma, struct moxart_dma_chan *mchans)
{
	struct moxart_dma_chan *mchan = &mchans[0];
	int i;

	for (i = 0; i < APB_DMA_MAX_CHANNEL; i++, mchan++) {
		mchan->ch_num = i;
		mchan->reg = (struct moxart_dma_reg *)(dma_base_addr + 0x80
			+ i*sizeof(struct moxart_dma_reg));
		mchan->callback = NULL;
		mchan->alloced = 0;
		mchan->callback_param = NULL;
		vchan_init(&mchan->vchan, dma);
		pr_debug("%s: mchans[%d]: mchan->ch_num=%d mchan->reg=%p\n",
			__func__, i, mchan->ch_num, mchan->reg);
	}
}

static irqreturn_t moxart_dma_interrupt(int irq, void *devid)
{
/*	struct moxart_dma_container *m = dev_get_drvdata(devid);*/
	struct moxart_dma_chan *mchan = &mdc->slave_chans[0];
	unsigned int i;
	union moxart_dma_reg_cfg mcfg;

	pr_debug("%s\n", __func__);

	for (i = 0; i < APB_DMA_MAX_CHANNEL; i++, mchan++) {
		if (mchan->alloced) {
			mcfg.ul = readl(&mchan->reg->cfg.ul);
			if (mcfg.ul & APB_DMA_FIN_INT_STS) {
				mcfg.ul &= ~APB_DMA_FIN_INT_STS;
				dma_cookie_complete(&mchan->tx_desc);
			}
			if (mcfg.ul & APB_DMA_ERR_INT_STS) {
				mcfg.ul &= ~APB_DMA_ERR_INT_STS;
				mchan->error_flag = 1;
			}
			if (mchan->callback) {
				pr_debug("%s: call callback for mchan=%p\n",
					__func__, mchan);
				mchan->callback(mchan->callback_param);
			}
			mchan->error_flag = 0;
			writel(mcfg.ul, &mchan->reg->cfg.ul);
		}
	}

	return IRQ_HANDLED;
}

static struct irqaction moxart_dma_irq = {
	.name       = "moxart-dma-engine",
	.flags      = IRQF_DISABLED,
	.handler    = moxart_dma_interrupt,
};

static int moxart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource res_dma;
	static void __iomem *dma_base_addr;
	int ret;
	unsigned int irq;

	mdc = devm_kzalloc(&pdev->dev, sizeof(*mdc), GFP_KERNEL);
	if (!mdc) {
		dev_err(&pdev->dev, "can't allocate DMA container\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(node, 0, &res_dma);
	if (ret) {
		dev_err(dev, "could not get DMA base resource\n");
		return ret;
	}

	irq = irq_of_parse_and_map(node, 0);

	dma_base_addr = devm_ioremap_resource(dev, &res_dma);
	if (IS_ERR(dma_base_addr)) {
		dev_err(dev, "%s: devm_ioremap_resource res_dma failed\n",
			__func__);
		return PTR_ERR(dma_base_addr);
	}

	mdc->ctlr = pdev->id;

	dma_cap_zero(mdc->dma_slave.cap_mask);
	dma_cap_set(DMA_SLAVE, mdc->dma_slave.cap_mask);

	moxart_dma_init(&mdc->dma_slave, &pdev->dev);
	moxart_dma_chan_init(dma_base_addr, &mdc->dma_slave, mdc->slave_chans);

	ret = dma_async_device_register(&mdc->dma_slave);
	platform_set_drvdata(pdev, mdc);

	setup_irq(irq, &moxart_dma_irq);

	dev_info(&pdev->dev, "finished %s IRQ=%d\n", __func__, irq);

	return ret;
}

static int moxart_remove(struct platform_device *pdev)
{
	struct moxart_dma_container *m = dev_get_drvdata(&pdev->dev);
	dma_async_device_unregister(&m->dma_slave);
	return 0;
}

static const struct of_device_id moxart_dma_match[] = {
	{ .compatible = "moxa,moxart-dma" },
	{ }
};

static struct platform_driver moxart_driver = {
	.probe		= moxart_probe,
	.remove		= moxart_remove,
	.driver = {
		.name			= "moxart-dma-engine",
		.owner			= THIS_MODULE,
		.of_match_table = moxart_dma_match,
	},
};

static int moxart_init(void)
{
	return platform_driver_register(&moxart_driver);
}
subsys_initcall(moxart_init);

static void __exit moxart_exit(void)
{
	platform_driver_unregister(&moxart_driver);
}
module_exit(moxart_exit);

MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");
MODULE_DESCRIPTION("MOXART DMA engine driver");
MODULE_LICENSE("GPL v2");
