/* Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#ifndef __MACH_APB_DMA_H
#define __MACH_APB_DMA_H

#define APB_DMA_MAX_CHANNEL		4
/*#define AHB_DMA_MAX_CHANNEL		8*/

union moxart_dma_reg_cfg {
	unsigned int ul;
#define APB_DMA_ENABLE				(1<<0)
#define APB_DMA_FIN_INT_STS			(1<<1)
#define APB_DMA_FIN_INT_EN			(1<<2)
#define APB_DMA_BURST_MODE			(1<<3)
#define APB_DMA_ERR_INT_STS			(1<<4)
#define APB_DMA_ERR_INT_EN			(1<<5)
#define APB_DMA_SOURCE_AHB			(1<<6)
#define APB_DMA_SOURCE_APB			0
#define APB_DMA_DEST_AHB			(1<<7)
#define APB_DMA_DEST_APB			0
#define APB_DMA_SOURCE_INC_0		0
#define APB_DMA_SOURCE_INC_1_4		(1<<8)
#define APB_DMA_SOURCE_INC_2_8		(2<<8)
#define APB_DMA_SOURCE_INC_4_16		(3<<8)
#define APB_DMA_SOURCE_DEC_1_4		(5<<8)
#define APB_DMA_SOURCE_DEC_2_8		(6<<8)
#define APB_DMA_SOURCE_DEC_4_16		(7<<8)
#define APB_DMA_SOURCE_INC_MASK		(7<<8)
#define APB_DMA_DEST_INC_0			0
#define APB_DMA_DEST_INC_1_4		(1<<12)
#define APB_DMA_DEST_INC_2_8		(2<<12)
#define APB_DMA_DEST_INC_4_16		(3<<12)
#define APB_DMA_DEST_DEC_1_4		(5<<12)
#define APB_DMA_DEST_DEC_2_8		(6<<12)
#define APB_DMA_DEST_DEC_4_16		(7<<12)
#define APB_DMA_DEST_INC_MASK		(7<<12)
#define APB_DMA_DEST_REQ_NO_MASK	(15<<16)
#define APB_DMA_DATA_WIDTH_MASK		(3<<20)
#define APB_DMA_DATA_WIDTH_4		0
#define APB_DMA_DATA_WIDTH_2		(1<<20)
#define APB_DMA_DATA_WIDTH_1		(2<<20)
#define APB_DMA_SOURCE_REQ_NO_MASK	(15<<24)
	struct {
		unsigned int enable:1;		/* enable DMA */
#define APB_DMAB_ENABLE				1
		unsigned int fin_int_sts:1;	/* finished interrupt status */
#define APB_DMAB_FIN_INT_STS		1
		unsigned int fin_int_en:1;	/* finished interrupt enable */
#define APB_DMAB_FIN_INT_EN			1
		unsigned int burst:1;		/* burst mode */
#define APB_DMAB_BURST_MODE			1
		unsigned int err_int_sts:1;	/* error interrupt status */
#define APB_DMAB_ERR_INT_STS		1
		unsigned int err_int_en:1;	/* error interrupt enable */
#define APB_DMAB_ERR_INT_EN			1
		unsigned int source_sel:1;	/* 0:APB (device),1:AHB (RAM) */
#define APB_DMAB_SOURCE_AHB			1
#define APB_DMAB_SOURCE_APB			0
		unsigned int dest_sel:1;	/* 0:APB,1:AHB */
#define APB_DMAB_DEST_AHB			1
#define APB_DMAB_DEST_APB			0
		unsigned int source_inc:3;
					/* 000:no increment
					   001:+1(busrt=0),+4 (burst=1)
					   010:+2(burst=0),+8 (burst=1)
					   011:+4(burst=0),+16(burst=1)
					   101:-1(burst=0),-4 (burst=1)
					   110:-2(burst=0),-8 (burst=1)
					   111:-4(burst=0),-16(burst=1) */
#define APB_DMAB_SOURCE_INC_0		0
#define APB_DMAB_SOURCE_INC_1_4		1
#define APB_DMAB_SOURCE_INC_2_8		2
#define APB_DMAB_SOURCE_INC_4_16	3
#define APB_DMAB_SOURCE_DEC_1_4		5
#define APB_DMAB_SOURCE_DEC_2_8		6
#define APB_DMAB_SOURCE_DEC_4_16	7
#define APB_DMAB_SOURCE_INC_MASK	7
		unsigned int reserved1:1;
		unsigned int dest_inc:3;
					/* 000:no increment
					   001:+1(busrt=0),+4 (burst=1)
					   010:+2(burst=0),+8 (burst=1)
					   011:+4(burst=0),+16(burst=1)
					   101:-1(burst=0),-4 (burst=1)
					   110:-2(burst=0),-8 (burst=1)
					   111:-4(burst=0),-16(burst=1) */
#define APB_DMAB_DEST_INC_0			0
#define APB_DMAB_DEST_INC_1_4		1
#define APB_DMAB_DEST_INC_2_8		2
#define APB_DMAB_DEST_INC_4_16		3
#define APB_DMAB_DEST_DEC_1_4		5
#define APB_DMAB_DEST_DEC_2_8		6
#define APB_DMAB_DEST_DEC_4_16		7
#define APB_DMAB_DEST_INC_MASK		7
		unsigned int reserved2:1;
		unsigned int dest_req_no:4;
			/* request signal select of dest */
			/* addr for DMA hwd handshake
			   0:no request/grant signal
			   1-15:request/grant signal */
#define APB_DMAB_DEST_REQ_NO_MASK	15
		unsigned int data_width:2;	/* data width of transfer */
					/* 00:word, 01:half, 10:byte */
#define APB_DMAB_DATA_WIDTH_MASK	3
#define APB_DMAB_DATA_WIDTH_4		0
#define APB_DMAB_DATA_WIDTH_2		1
#define APB_DMAB_DATA_WIDTH_1		2
		unsigned int reserved3:2;
		unsigned int source_req_no:4;
			/* request signal select of dest */
			/* addr for DMA hwd handshake
			   0:no request/grant signal
			   1-15:request/grant signal */
#define APB_DMAB_SOURCE_REQ_NO_MASK	15
		unsigned int reserved4:4;
	} bits;
};

struct moxart_dma_reg {
	unsigned int source_addr;
	unsigned int dest_addr;
	unsigned int cycles;	/* is depended on burst mode */
#define APB_DMA_CYCLES_MASK	0x00ffffff
	union moxart_dma_reg_cfg cfg;
};

#endif	/* __MACH_APB_DMA_H */
