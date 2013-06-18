/*
 * MOXA ART MMC host driver.
 *
 * Copyright (C) 2013 Jonas Jensen
 *
 * Jonas Jensen <jonas.jensen@gmail.com>
 *
 * Based on code from
 * Moxa Technology Co., Ltd. <www.moxa.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _MOXART_H
#define _MOXART_H

extern bool moxart_filter_fn(struct dma_chan *chan, void *param);

#define MSD_CMD_REG			0		/* in bytes */
#define MSD_ARG_REG			4
#define MSD_RESP0_REG		8
#define MSD_RESP1_REG		0x0c
#define MSD_RESP2_REG		0x10
#define MSD_RESP3_REG		0x14
#define MSD_RESP_CMD_REG	0x18
#define MSD_DATA_CTRL_REG	0x1c
#define MSD_DATA_TIMER_REG	0x20
#define MSD_DATA_LEN_REG	0x24
#define MSD_STATUS_REG		0x28
#define MSD_CLEAR_REG		0x2c
#define MSD_INT_MASK_REG	0x30
#define MSD_POWER_CTRL_REG	0x34
#define MSD_CLOCK_CTRL_REG	0x38
#define MSD_BUS_WIDTH_REG	0x3c
#define MSD_DATA_WIN_REG	0x40
#define MSD_FEATURE_REG		0x44
#define MSD_REVISION_REG	0x48

#define MMC_RSP_SHORT   (1 << 0)
#define MMC_RSP_LONG    (2 << 0)
#define MMC_RSP_MASK    (3 << 0)
#define MMC_ERR_NONE	0
#define MMC_ERR_TIMEOUT 1
#define MMC_MODE_MMC	0
#define MMC_MODE_SD		1
#define MMC_ERR_BADCRC  2
#define MMC_VDD_360     23

#define MSD_RETRY_COUNT		10

#define SD_APP_SET_BUS_WIDTH	6   /* ac   [1:0] bus width    R1  */

struct moxart_reg {
	unsigned int	command;
#define MSD_SDC_RST			(1<<10)
#define MSD_CMD_EN			(1<<9)
#define MSD_APP_CMD			(1<<8)
#define MSD_LONG_RSP			(1<<7)
#define MSD_NEED_RSP			(1<<6)
#define MSD_CMD_IDX_MASK		0x3f
	unsigned int	argument;
	unsigned int	response0;
	unsigned int	response1;
	unsigned int	response2;
	unsigned int	response3;
	unsigned int	response_command;
#define MSD_RSP_CMD_APP			(1<<6)
#define MSD_RSP_CMD_IDX_MASK		0x3f
	unsigned int	data_control;
#define MSD_DATA_EN			(1<<6)
#define MSD_DMA_EN			(1<<5)
#define MSD_DATA_WRITE			(1<<4)
#define MSD_BLK_SIZE_MASK		0x0f
	unsigned int	data_timer;
	unsigned int	data_length;
#define MSD_DATA_LEN_MASK		0xffffff
	unsigned int	status;
#define MSD_WRITE_PROT			(1<<12)
#define MSD_CARD_DETECT			(1<<11)
/* 1-10 below can be sent to interrupt or clear register */
#define MSD_CARD_CHANGE			(1<<10)
#define MSD_FIFO_ORUN			(1<<9)
#define MSD_FIFO_URUN			(1<<8)
#define MSD_DATA_END			(1<<7)
#define MSD_CMD_SENT			(1<<6)
#define MSD_DATA_CRC_OK			(1<<5)
#define MSD_RSP_CRC_OK			(1<<4)
#define MSD_DATA_TIMEOUT		(1<<3)
#define MSD_RSP_TIMEOUT			(1<<2)
#define MSD_DATA_CRC_FAIL		(1<<1)
#define MSD_RSP_CRC_FAIL		(1<<0)
	unsigned int	clear;
	unsigned int	interrupt_mask;
	unsigned int	power_control;
#define MSD_SD_POWER_ON			(1<<4)
#define MSD_SD_POWER_MASK		0x0f
	unsigned int	clock_control;
#define MSD_CLK_DIS			(1<<8)
#define MSD_CLK_SD			(1<<7)
#define MSD_CLK_DIV_MASK		0x7f
	unsigned int	bus_width;
#define MSD_WIDE_BUS_SUPPORT		(1<<3)
#define MSD_WIDE_BUS			(1<<2)	/* bus width=4 */
#define MSD_SINGLE_BUS			(1<<0)	/* bus width=1 */
	unsigned int	data_window;
	unsigned int	feature;
#define MSD_CPRM_FUNCTION		(1<<8)
	unsigned int	revision;
};

struct moxart_host {
	spinlock_t				lock;
	struct moxart_reg		*reg;
	phys_addr_t				reg_phys;

	struct dma_chan			*dma_chan_rx;
	struct dma_chan			*dma_chan_tx;
	struct dma_chan			*dma_chan_cur;
	unsigned int			dma_direction;
	bool					have_dma;
	struct completion		dma_complete;
	struct completion		pio_complete;

	struct mmc_host			*mmc;
	struct mmc_request		*mrq;

	struct clk				*clk;

	struct scatterlist		*cur_sg;
	unsigned int			num_sg;
	unsigned int			remain;
	int						size;

	unsigned int			wait_for;
	unsigned int			timeout;
	long					sysclk;
	bool					removed;
};

#define MSD_FIFO_LENW	4	/* 4 words, total 4 * 4 = 16 bytes */
#define MSD_FIFO_LENB	16	/* 16 bytes */

#endif	/* _MOXART_H */
