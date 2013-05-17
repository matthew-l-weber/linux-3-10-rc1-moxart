/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/sizes.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>

#include <asm/dma.h>
#include <asm/irq.h>

#include "sdhci-moxart.h"

#define APB_CLK					48000000
#define MSD_SUPPORT_GET_CLOCK
#define DMA_FIFO_LEN_FORCE		0
#define APB_DMA_SD_REQ_NO		5
#define MIN_POWER (MMC_VDD_360 - MSD_SD_POWER_MASK)

static inline void moxart_init_sg(struct moxart_host *host,
	struct mmc_data *data)
{
	host->cur_sg = data->sg;
	host->num_sg = data->sg_len;
	host->remain = host->cur_sg->length;

	if (host->remain > host->size)
		host->remain = host->size;

	data->error = MMC_ERR_NONE;
}

static inline int moxart_next_sg(struct moxart_host *host)
{
	int remain;
	struct mmc_data *data = host->mrq->cmd->data;

	host->cur_sg++;
	host->num_sg--;

	if (host->num_sg > 0) {
		host->remain = host->cur_sg->length;
		remain = host->size - data->bytes_xfered;
		if (remain > 0 && remain < host->remain)
			host->remain = remain;
	}

	return host->num_sg;
}

static void moxart_send_command(struct moxart_host *host,
	struct mmc_command *cmd)
{
	unsigned int status, cmdctrl;
	int retry = 0;

	pr_debug("%s: cmd->opcode=%d\n", __func__, cmd->opcode);

	cmd->error = MMC_ERR_TIMEOUT;

	writel(MSD_RSP_TIMEOUT | MSD_RSP_CRC_OK |
		MSD_RSP_CRC_FAIL | MSD_CMD_SENT, &host->reg->clear);
	writel(cmd->arg, &host->reg->argument);

	cmdctrl = cmd->opcode & MSD_CMD_IDX_MASK;
	if (cmdctrl == SD_APP_SET_BUS_WIDTH || cmdctrl == SD_APP_OP_COND ||
		cmdctrl == SD_APP_SEND_SCR || cmdctrl == SD_APP_SD_STATUS ||
		cmdctrl == SD_APP_SEND_NUM_WR_BLKS)
		cmdctrl |= MSD_APP_CMD;

	if (cmd->flags & MMC_RSP_136)
		cmdctrl |= (MSD_LONG_RSP | MSD_NEED_RSP);
	else
		cmdctrl |= MSD_NEED_RSP;

	writel(cmdctrl | MSD_CMD_EN, &host->reg->command);

	while (retry++ < MSD_RETRY_COUNT) {
		udelay(10);
		status = readl(&host->reg->status);
		if (status & MSD_CARD_DETECT) {
			pr_debug("%s: MSD_CARD_DETECT\n", __func__);
			cmd->error = MMC_ERR_TIMEOUT;
			break;
		}
		if (cmdctrl & MSD_NEED_RSP) {
			pr_debug("%s: MSD_NEED_RSP\n", __func__);
			if (status & MSD_RSP_TIMEOUT) {
				pr_debug("%s: MSD_RSP_TIMEOUT\n", __func__);
				writel(MSD_RSP_TIMEOUT, &host->reg->clear);
				cmd->error = MMC_ERR_TIMEOUT;
				break;
			}
			if ((cmd->flags & MMC_RSP_CRC) &&
				(status & MSD_RSP_CRC_FAIL)) {
				pr_debug("%s: MSD_RSP_CRC_FAIL\n", __func__);
				writel(MSD_RSP_CRC_FAIL, &host->reg->clear);
				cmd->error = MMC_ERR_BADCRC;
				break;
			}
			if (status & MSD_RSP_CRC_OK) {
				pr_debug("%s: MSD_RSP_CRC_OK\n", __func__);
				writel(MSD_RSP_CRC_OK, &host->reg->clear);

				if (cmd->flags & MMC_RSP_136) {
					cmd->resp[3] =
						readl(&host->reg->response0);
					cmd->resp[2] =
						readl(&host->reg->response1);
					cmd->resp[1] =
						readl(&host->reg->response2);
					cmd->resp[0] =
						readl(&host->reg->response3);
				} else {
					cmd->resp[0] =
						readl(&host->reg->response0);
				}

				cmd->error = MMC_ERR_NONE;
				break;
			}
		} else {
			pr_debug("%s: !(cmdctrl & MSD_NEED_RSP)\n", __func__);
			if (status & MSD_CMD_SENT) {
				writel(MSD_CMD_SENT, &host->reg->clear);
				cmd->error = MMC_ERR_NONE;
				break;
			}
		}
	}

	if (retry >= (MSD_RETRY_COUNT - 1)) {
		/*	this seems to happen a lot on or after CMD25
			(MMC_WRITE_MULTIPLE_BLOCK) with more than 4 blocks
			(>4096), possibly because the transfer is too big
			or takes too long to complete.
			when this happens the controller is usually rendered
			unresponsive and can only be recovered with reboot. */
		pr_debug("%s: WARNING! no valid status found!\n", __func__);
	}
}

static void moxart_dma_complete(void *param)
{
	struct moxart_host *host = param;

	pr_debug("%s: host=%p\n", __func__, host);
	complete(&host->dma_complete);
}

static void moxart_transfer_dma(struct mmc_data *data, struct moxart_host *host)
{
	unsigned int len, direction_dev;
	struct dma_async_tx_descriptor *desc = NULL;

	if (host->size == data->bytes_xfered)
		return;

	direction_dev = (data->flags & MMC_DATA_WRITE) ?
		DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;

	len = dma_map_sg(host->dma_chan_cur->device->dev, data->sg,
		data->sg_len, host->dma_direction);
	/*	because dma_map_sg takes both sg and sg_len as arguments
		(and maps the entire list of buffers) data->sg does not
		have to be incremented between calls (as is required
		in moxart_transfer_pio) */

	if (len > 0) {
		/* host->dma_active = true;*/
		desc = dmaengine_prep_slave_sg(host->dma_chan_cur, data->sg,
			len, direction_dev,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	} else {
		pr_err("%s: dma_map_sg returned zero length\n",
			__func__);
	}

	if (desc) {
		desc->callback = moxart_dma_complete;
		desc->callback_param = host;
		/* cookie = */ dmaengine_submit(desc);
		dma_async_issue_pending(host->dma_chan_cur);
	}

	data->bytes_xfered += host->remain;
}

/* with DMA enabled this is still used but only for SD_APP_SEND_SCR */
static void moxart_transfer_pio(struct moxart_host *host)
{
	unsigned char *buffer;
	unsigned int wcnt, i;
	struct mmc_data *data = host->mrq->cmd->data;

	if (host->size == data->bytes_xfered)
		goto transfer_complete;

	buffer = sg_virt(host->cur_sg);
	wcnt = host->remain >> 2;

	if (data->flags & MMC_DATA_WRITE) {
		for (i = 0; i < wcnt; i++, buffer += 4) {
			/*pr_debug("%s: MMC_DATA_WRITE buf=%x i=%d wcnt=%d\n",
				__func__, *(unsigned int *)buffer, i, wcnt);*/
			writel(*(unsigned int *)buffer,
				&host->reg->data_window);
			udelay(10);
		}
	} else {
		for (i = 0; i < wcnt; i++, buffer += 4) {
			if (data->mrq->cmd->opcode == SD_APP_SEND_SCR) {
				/* byte order reversed only when reading SCR */
				*(unsigned int *)buffer = ioread32be(
					&host->reg->data_window);
			} else {
				*(unsigned int *)buffer =
					readl(&host->reg->data_window);
			}
			/*pr_debug("%s: MMC_DATA_READ buf=%x i=%d wcnt=%d\n",
				__func__, *(unsigned int *)buffer, i, wcnt);*/
			udelay(10);
		}
	}
	wcnt <<= 2;
	host->remain -= wcnt;
	data->bytes_xfered += wcnt;

	if (host->size != data->bytes_xfered) {
		moxart_next_sg(host);
		/*	goto transfer_start is not needed, this function
			will be called again from interrupt.

			compare this with DMA where sg increment is
			redundant thanks to dma_map_sg */
	} else {
		complete(&host->pio_complete);
	}

transfer_complete:
	pr_debug("%s: host->size=%d host->remain=%u\n", __func__,
		host->size, host->remain);
}

static void moxart_prepare_data(struct moxart_host *host)
{
	struct mmc_data *data = host->mrq->cmd->data;
	unsigned int timeout, datactrl;
	int blksz_bits;

	pr_debug("%s:\n", __func__);

	if (!data)
		return;

	host->size = data->blocks * data->blksz;
	blksz_bits = ffs(data->blksz) - 1;
	/*	find the first (from left) bit set, ffs(512) = 10
		this is the 10th bit from right (0x1 << 9) */
	BUG_ON(1 << blksz_bits != data->blksz);

	moxart_init_sg(host, data);

	timeout = (host->mmc->f_max / 1000) * (data->timeout_ns / 1000);
	timeout *= 2;

	datactrl = (blksz_bits & MSD_BLK_SIZE_MASK) | MSD_DATA_EN;

	if (data->flags & MMC_DATA_WRITE)
		datactrl |= MSD_DATA_WRITE;

	if (((host->size > MSD_FIFO_LENB) || DMA_FIFO_LEN_FORCE)
		&& host->have_dma) {
		datactrl |= MSD_DMA_EN;
	}

	pr_debug("%s: blocks=%d blksz=%d datactrl=0x%08x\n",
		__func__, data->blocks,	data->blksz, datactrl);
	pr_debug("%s: timeout=%u timeout_ns=%u\n",
		__func__, timeout, data->timeout_ns);

	writel(timeout, &host->reg->data_timer);
	writel(host->size, &host->reg->data_length);
	writel(datactrl, &host->reg->data_control);
}

static void moxart_transfer_check(struct mmc_data *data,
	struct moxart_host *host)
{
	unsigned int status, count = 0;

	pr_debug("%s\n", __func__);

	while (1) {
		udelay(10);
		status = readl(&host->reg->status);
		if (status & (MSD_DATA_CRC_OK | MSD_DATA_CRC_FAIL
			| MSD_DATA_END)	|| count > 10)
			break;
		pr_debug("%s: waiting for status=%08x ..\n", __func__, status);
		count++;
	}
	if (status & MSD_DATA_CRC_OK) {
		pr_debug("%s: MSD_DATA_CRC_OK\n", __func__);
		writel(MSD_DATA_CRC_OK, &host->reg->clear);
	}
	if (status & MSD_DATA_CRC_FAIL) {
		pr_debug("%s: MSD_DATA_CRC_FAIL\n", __func__);
		writel(MSD_DATA_CRC_FAIL, &host->reg->clear);
		data->error = MMC_ERR_TIMEOUT;
	}
	if (status & MSD_DATA_END) {
		pr_debug("%s: MSD_DATA_END\n", __func__);
		writel(MSD_DATA_END, &host->reg->clear);
	}
	if (status & MSD_DATA_TIMEOUT) {
		pr_debug("%s: MSD_DATA_TIMEOUT\n", __func__);
		writel(MSD_DATA_TIMEOUT, &host->reg->clear);
	}
}

static void moxart_card_change(struct moxart_host *host)
{
	int delay;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (readl(&host->reg->status) & MSD_CARD_DETECT) {
		pr_debug("%s: card removed\n", __func__);
		if (host->have_dma && host->size > MSD_FIFO_LENB) {
			pr_debug("%s: call dmaengine_terminate_all\n",
				__func__);
			dmaengine_terminate_all(host->dma_chan_rx);
			dmaengine_terminate_all(host->dma_chan_tx);
		}
		host->removed = true;
		delay = 0;
	} else {
		pr_debug("%s: card inserted\n", __func__);
		host->removed = false;
		delay = 500;
	}

	/*	clearing FIFO interrupts here does not stop a follow up
		MSD_FIFO_*RUN after MSD_CARD_CHANGE. instead, check
		host->mrq != NULL in moxart_irq to avoid unnecessary calls
		to moxart_transfer_pio
		if this happens during transfer the mmc_request in
		moxart_request should still be valid
		(which is why host->mrq can be set NULL here) */
	host->mrq = NULL;
	writel(MSD_CARD_CHANGE | MSD_FIFO_ORUN | MSD_FIFO_URUN,
		&host->reg->clear);
	writel(MSD_CARD_CHANGE, &host->reg->interrupt_mask);

	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: call mmc_detect_change\n", __func__);
	mmc_detect_change(host->mmc, msecs_to_jiffies(delay));
}


static void moxart_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct moxart_host *host = mmc_priv(mmc);
	unsigned long dma_time, pio_time, flags;
	unsigned int status;

	pr_debug("%s:\n", __func__);

	spin_lock_irqsave(&host->lock, flags);

	init_completion(&host->dma_complete);
	init_completion(&host->pio_complete);

	host->mrq = mrq;

	if (readl(&host->reg->status) & MSD_CARD_DETECT) {
		/* card is removed if no card inserted, return timeout error */
		mrq->cmd->error = MMC_ERR_TIMEOUT;
		goto request_done;
	}

	moxart_prepare_data(host);
	moxart_send_command(host, host->mrq->cmd);

	if (mrq->cmd->data) {
		/* only use DMA/PIO (and wait) when there is data */
		if (((host->size > MSD_FIFO_LENB) || DMA_FIFO_LEN_FORCE)
			&& host->have_dma) {

			writel(MSD_CARD_CHANGE, &host->reg->interrupt_mask);

			spin_unlock_irqrestore(&host->lock, flags);

			host->dma_direction = (mrq->cmd->data->flags
				& MMC_DATA_WRITE) ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE;
			host->dma_chan_cur = (mrq->cmd->data->flags
				& MMC_DATA_WRITE) ?
				host->dma_chan_tx : host->dma_chan_rx;
			moxart_transfer_dma(mrq->cmd->data, host);

			dma_time = wait_for_completion_interruptible_timeout(
				&host->dma_complete, host->timeout);
			pr_debug("%s: dma_time=%lu (DMA wait time)\n",
				__func__, dma_time);

			dma_unmap_sg(host->dma_chan_cur->device->dev,
				mrq->cmd->data->sg,	mrq->cmd->data->sg_len,
				host->dma_direction);

			spin_lock_irqsave(&host->lock, flags);
		} else {

			writel(MSD_FIFO_URUN | MSD_FIFO_ORUN | MSD_CARD_CHANGE,
				&host->reg->interrupt_mask);

			status = readl(&host->reg->status);
			pr_debug("%s: status=%08x\n", __func__, status);

			spin_unlock_irqrestore(&host->lock, flags);

			/* PIO transfer started from interrupt */
			pio_time = wait_for_completion_interruptible_timeout(
				&host->pio_complete, host->timeout);
			pr_debug("%s: pio_time=%lu (PIO wait time)\n",
				__func__, pio_time);

			spin_lock_irqsave(&host->lock, flags);
		}

		/*	removed during transfer?
			(note that interrupts were just enabled..) */
		if (host->removed) {
			pr_debug("%s: host removed during transfer!\n",
				__func__);
			mrq->cmd->error = MMC_ERR_TIMEOUT;
	    }

		moxart_transfer_check(mrq->cmd->data, host);

		if (mrq->cmd->data->stop)
			moxart_send_command(host, mrq->cmd->data->stop);
	}

request_done:
	spin_unlock_irqrestore(&host->lock, flags);
	mmc_request_done(host->mmc, mrq);
}

static irqreturn_t moxart_irq(int irq, void *devid)
{
	struct moxart_host *host = (struct moxart_host *)devid;
	unsigned int status;
	unsigned long flags;

	status = readl(&host->reg->status);

	pr_debug("%s: host=%p status=%08x\n", __func__, host, status);

	if (status & MSD_CARD_CHANGE) {
		pr_debug("%s: call moxart_card_change\n", __func__);
		moxart_card_change(host);
	}
	if (status & (MSD_FIFO_ORUN | MSD_FIFO_URUN) && host->mrq) {
		writel(status & (MSD_FIFO_ORUN | MSD_FIFO_URUN),
			&host->reg->clear);
		pr_debug("%s: call moxart_transfer_pio\n", __func__);
		spin_lock_irqsave(&host->lock, flags);
		moxart_transfer_pio(host);
		spin_unlock_irqrestore(&host->lock, flags);
	}

	return IRQ_HANDLED;
}

static void moxart_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct moxart_host *host = mmc_priv(mmc);
	unsigned long flags;
	unsigned short power;

	spin_lock_irqsave(&host->lock, flags);
	if (ios->clock) {
		int div;
#ifdef MSD_SUPPORT_GET_CLOCK
		div = (host->sysclk / (host->mmc->f_max * 2)) - 1;
#else
		div = (APB_CLK / (host->mmc->f_max * 2)) - 1;
#endif

		if (div > MSD_CLK_DIV_MASK)
			div = MSD_CLK_DIV_MASK;
		else if (div < 0)
			div = 0;

		div |= MSD_CLK_SD;
		writel(div, &host->reg->clock_control);
	} else if (!(readl(&host->reg->clock_control) & MSD_CLK_DIS)) {
		/* ensure that the clock is off. */
		writel(readl(&host->reg->clock_control) | MSD_CLK_DIS,
			&host->reg->clock_control);
	}

	if (ios->power_mode == MMC_POWER_OFF) {
		writel(readl(&host->reg->power_control) & ~MSD_SD_POWER_ON,
			&host->reg->power_control);
	} else {
		if (ios->vdd < MIN_POWER)
			power = 0;
		else
			power = ios->vdd - MIN_POWER;

		writel(MSD_SD_POWER_ON | (unsigned int) power,
			&host->reg->power_control);
	}

	if (ios->bus_width == MMC_BUS_WIDTH_1) {
		/*dev_info(mmc_dev(mmc), "%s: set bus MSD_SINGLE_BUS width\n"
			, __func__);*/
		writel(MSD_SINGLE_BUS, &host->reg->bus_width);
	} else {
		/*dev_info(mmc_dev(mmc), "%s: set bus MSD_WIDE_BUS width\n",
		__func__);*/
		writel(MSD_WIDE_BUS, &host->reg->bus_width);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static void moxart_get_sysclk(struct moxart_host *host, void __iomem *reg_pmu)
{
	unsigned int mul, val, div;
	mul = (readl(reg_pmu + 0x30) >> 3) & 0x1ff;
	val = (readl(reg_pmu + 0x0c) >> 4) & 0x7;
	switch (val) {
	case 0:
		div = 2;
		break;
	case 1:
		div = 3;
		break;
	case 2:
		div = 4;
		break;
	case 3:
		div = 6;
		break;
	case 4:
		div = 8;
		break;
	default:
		div = 2;
		break;
	}
	host->sysclk = (38684 * mul + 10000) / (div * 10000);
	host->sysclk = (host->sysclk * 1000000) / 2;
	dev_dbg(mmc_dev(host->mmc),
		"%s: host->sysclk=%d mul=%d div=%d val=%d\n",
		__func__, host->sysclk, mul, div, val);
	dev_info(mmc_dev(host->mmc),
		"%s: host->sysclk=%d mul=%d div=%d val=%d\n",
		__func__, host->sysclk, mul, div, val);
	/* host->sysclk=77500000 mul=80 div=2 val=0 */
}

static int moxart_get_ro(struct mmc_host *mmc)
{
	int ret;
	struct moxart_host *host = mmc_priv(mmc);

	(readl(&host->reg->status) & MSD_WRITE_PROT) ? (ret = 1) : (ret = 0);
	return ret;
}

static struct mmc_host_ops moxart_ops = {
	.request = moxart_request,
	.set_ios = moxart_set_ios,
	.get_ro = moxart_get_ro,
};

static int moxart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource res_pmu, res_mmc;
	struct mmc_host *mmc;
	struct moxart_host *host = NULL;
	void __iomem *reg_mmc;
	void __iomem *reg_pmu;
	dma_cap_mask_t mask;
	int ret;
	struct dma_slave_config cfg;
	unsigned int dma_chan_rx_req = 1;
	unsigned int dma_chan_tx_req = 0;
	unsigned int irq;

	mmc = mmc_alloc_host(sizeof(struct moxart_host), &pdev->dev);
	if (!mmc) {
		pr_err("%s: mmc_alloc_host failed\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	ret = of_address_to_resource(node, 0, &res_mmc);
	if (ret) {
		dev_err(dev, "%s: could not get MMC base resource\n", __func__);
		goto out;
	}
	ret = of_address_to_resource(node, 1, &res_pmu);
	if (ret) {
		dev_err(dev, "%s: could not get PMU base resource\n", __func__);
		goto out;
	}

	irq = irq_of_parse_and_map(node, 0);

	reg_mmc = devm_ioremap_resource(dev, &res_mmc);
	if (IS_ERR(reg_mmc)) {
		dev_err(dev, "%s: devm_ioremap_resource res_mmc failed\n",
			__func__);
		return PTR_ERR(reg_mmc);
	}
	/*	use ioremap here instead of devm_ioremap_resource
		gpio will request the memory region first (postcore_initcall)
		and doing it here again would fail */
	reg_pmu = ioremap(res_pmu.start, resource_size(&res_pmu));
	if (IS_ERR(reg_pmu)) {
		dev_err(dev, "%s: devm_ioremap_resource res_pmu failed\n",
			__func__);
		return PTR_ERR(reg_pmu);
	}

	mmc->ops = &moxart_ops;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	/*	hardware does not support MMC_CAP_SD_HIGHSPEED
		CMD6 will timeout and make things not work */
	mmc->f_min = 400000;
	mmc->f_max = 25000000;
	mmc->ocr_avail = 0xffff00;	/* support 2.0v - 3.6v power */
	mmc->max_segs = 32;
	mmc->max_blk_size = 512;
	/*mmc->max_req_size = PAGE_CACHE_SIZE * mmc->max_segs;*/
	mmc->max_blk_count = mmc->max_req_size / mmc->max_blk_size;
	mmc->max_seg_size = mmc->max_req_size;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->reg = (struct moxart_reg *)reg_mmc;
	host->reg_phys = res_mmc.start;
	host->timeout = msecs_to_jiffies(1000);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	host->clk = devm_clk_get(dev, "sys_clk");
	if (IS_ERR(host->clk)) {
		dev_warn(dev, "can not get sysclk\n");
	}
	else {
		dev_info(dev, "sysclk=%lu\n", clk_round_rate(host->clk, 0));
	}
	
#ifdef MSD_SUPPORT_GET_CLOCK
	moxart_get_sysclk(host, reg_pmu);
#endif

	spin_lock_init(&host->lock);

	/* disable all interrupt */
	writel(0, &host->reg->interrupt_mask);

	/* reset chip */
	writel(MSD_SDC_RST, &host->reg->command);

	/* wait for reset finished */
	while (readl(&host->reg->command) & MSD_SDC_RST)
		udelay(10);

	host->dma_chan_tx = dma_request_channel(mask, moxart_filter_fn,
		(void *)&dma_chan_tx_req);
	host->dma_chan_rx = dma_request_channel(mask, moxart_filter_fn,
		(void *)&dma_chan_rx_req);
	dev_dbg(&host->mmc->class_dev, "%s: using 2 DMA channels rx=%p tx=%p\n",
		__func__, host->dma_chan_rx, host->dma_chan_tx);

	if (!host->dma_chan_rx || !host->dma_chan_tx) {
		host->have_dma = false;
		mmc->max_blk_count = 1;
	} else {
		cfg.slave_id = APB_DMA_SD_REQ_NO;
		cfg.direction = DMA_MEM_TO_DEV;
		cfg.src_addr = 0;
		cfg.dst_addr = (unsigned int)host->reg_phys + MSD_DATA_WIN_REG;
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dmaengine_slave_config(host->dma_chan_tx, &cfg);

		cfg.slave_id = APB_DMA_SD_REQ_NO;
		cfg.direction = DMA_DEV_TO_MEM;
		cfg.src_addr = (unsigned int)host->reg_phys + MSD_DATA_WIN_REG;
		cfg.dst_addr = 0;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dmaengine_slave_config(host->dma_chan_rx, &cfg);

		host->have_dma = true;
		mmc->max_blk_count = 16;
		/*	there seems to be a max size on transfers so
			set max_blk_count low for both DMA and PIO

			sending large chunks result either in timeout
			or render the MMC controller unresponsive
			(status register 0 on consecutive read retries,
			also see comments in moxart_send_command)

			obviously, DMA is quicker and can handle
			larger chunks but setting it higher than 16
			can still bug the controller */
	}

	ret = request_irq(irq, moxart_irq,
		IRQF_DISABLED, "moxart-mmc", host);

	if (ret)
		goto out;

	dev_set_drvdata(&pdev->dev, mmc);
	mmc_add_host(mmc);

	dev_info(mmc_dev(host->mmc), "finished %s IRQ=%d\n",
		__func__, irq);
	return 0;

out:
	if (mmc)
		mmc_free_host(mmc);
	return ret;
}

static void moxart_release_dma(struct moxart_host *host)
{
	if (host->dma_chan_tx) {
		struct dma_chan *chan = host->dma_chan_tx;
		host->dma_chan_tx = NULL;
		dma_release_channel(chan);
	}
	if (host->dma_chan_rx) {
		struct dma_chan *chan = host->dma_chan_rx;
		host->dma_chan_rx = NULL;
		dma_release_channel(chan);
	}
}

static int moxart_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = dev_get_drvdata(&pdev->dev);
	struct moxart_host *host = mmc_priv(mmc);
	struct resource *res_irq;

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 2);

	dev_set_drvdata(&pdev->dev, NULL);

	if (mmc) {
		moxart_release_dma(host);
		mmc_remove_host(mmc);

		writel(0, &host->reg->interrupt_mask);
		writel(0, &host->reg->power_control);
		writel(readl(&host->reg->clock_control) | MSD_CLK_DIS,
			&host->reg->clock_control);

		free_irq(res_irq->start, host);

		mmc_free_host(mmc);
	}
	return 0;
}

static const struct of_device_id moxart_mmc_match[] = {
	{ .compatible = "moxa,moxart-mmc" },
	{ }
};

static struct platform_driver moxart_mmc_driver = {
	.probe      = moxart_probe,
	.remove     = moxart_remove,
	.driver     = {
		.name			= "sdhci-moxart",
		.owner			= THIS_MODULE,
		.of_match_table	= moxart_mmc_match,
	},
};
module_platform_driver(moxart_mmc_driver);

MODULE_ALIAS("platform:sdhci-moxart");
MODULE_DESCRIPTION("MOXART SDHCI driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");

