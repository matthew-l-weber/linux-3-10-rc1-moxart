/* MOXART Ethernet (RTL8201CP) device driver (based on MOXA sources)
 * Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef _MOXART_ETHERNET_H
#define _MOXART_ETHERNET_H

#define TX_DESC_NUM         64
#define TX_DESC_NUM_MASK    (TX_DESC_NUM-1)
#define RX_DESC_NUM         64
#define RX_DESC_NUM_MASK    (RX_DESC_NUM-1)
#define TX_BUF_SIZE         1600
#define RX_BUF_SIZE         1600

struct tx_desc_t {
	union {
		unsigned int ui;
#define TXDMA_OWN           (1<<31)
#define TXPKT_EXSCOL        (1<<1)
#define TXPKT_LATECOL       (1<<0)
		struct {
			/* is aborted due to late collision */
			unsigned int tx_pkt_late_col:1;

			/* is aborted after 16 collisions */
			unsigned int rx_pkt_exs_col:1;

			unsigned int reserved1:29;

			/* is owned by the MAC controller */
			unsigned int tx_dma_own:1;
		} ubit;
	} txdes0;
	union {
		unsigned int ui;
#define EDOTR               (1<<31)
#define TXIC                (1<<30)
#define TX2FIC              (1<<29)
#define FTS                 (1<<28)
#define LTS                 (1<<27)
#define TXBUF_SIZE_MASK     0x7ff
#define TXBUF_SIZE_MAX      (TXBUF_SIZE_MASK+1)
		struct {
			/* transmit buffer size in byte */
			unsigned int tx_buf_size:11;

			unsigned int reserved2:16;

			/* is the last descriptor of a Tx packet */
			unsigned int lts:1;

			/* is the first descriptor of a Tx packet */
			unsigned int fts:1;

			/* transmit to FIFO interrupt on completion */
			unsigned int tx2_fic:1;

			/* transmit interrupt on completion */
			unsigned int tx_ic:1;

			/* end descriptor of transmit ring */
			unsigned int edotr:1;
		} ubit;
	} txdes1;
	struct {
		/* transmit buffer physical base address */
		unsigned int phy_tx_buf_baseaddr;

		/* transmit buffer virtual base address */
		unsigned char *virt_tx_buf_baseaddr;
	} txdes2;
};

struct rx_desc_t {
	union {
		unsigned int ui;
#define RXDMA_OWN           (1<<31)
#define FRS                 (1<<29)
#define LRS                 (1<<28)
#define RX_ODD_NB           (1<<22)
#define RUNT                (1<<21)
#define FTL                 (1<<20)
#define CRC_ERR             (1<<19)
#define RX_ERR              (1<<18)
#define BROADCAST_RXDES0    (1<<17)
#define MULTICAST_RXDES0    (1<<16)
#define RFL_MASK        0x7ff
#define RFL_MAX             (RFL_MASK+1)
		struct {
			/* receive frame length */
			unsigned int recv_frame_len:11;
			unsigned int reserved1:5;

			/* multicast frame */
			unsigned int multicast:1;

			/* broadcast frame */
			unsigned int broadcast:1;
			unsigned int rx_err:1;         /* receive error */
			unsigned int crc_err:1;        /* CRC error */
			unsigned int ftl:1;           /* frame too long */

			/* runt packet, less than 64 bytes */
			unsigned int runt:1;

			/* receive odd nibbles */
			unsigned int rx_odd_nb:1;
			unsigned int reserved2:5;

			/* last receive segment descriptor */
			unsigned int lrs:1;

			/* first receive segment descriptor */
			unsigned int frs:1;

			unsigned int reserved3:1;
			unsigned int rx_dma_own:1;      /* RXDMA onwership */
		} ubit;
	} rxdes0;
	union {
		unsigned int ui;
#define EDORR               (1<<31)
#define RXBUF_SIZE_MASK     0x7ff
#define RXBUF_SIZE_MAX      (RXBUF_SIZE_MASK+1)
		struct {
			/* receive buffer size */
			unsigned int rx_buf_size:11;

			unsigned int reserved4:20;

			/* end descriptor of receive ring */
			unsigned int edorr:1;
		} ubit;
	} rxdes1;
	struct {
		/* receive buffer physical base address */
		unsigned int phy_rx_buf_baseaddr;

		/* receive buffer virtual base address */
		unsigned char *virt_rx_buf_baseaddr;
	} rxdes2;
};

struct mac_control_reg_t {
	unsigned int isr;	/* interrupt status, 0x0 */

/* RXDMA has received packets into RX buffer successfully */
#define RPKT_FINISH         (1<<0)

/* receive buffer unavailable */
#define NORXBUF             (1<<1)

/* TXDMA has moved data into the TX FIFO */
#define XPKT_FINISH         (1<<2)

/* transmit buffer unavailable */
#define NOTXBUF             (1<<3)

/* packets transmitted to ethernet successfully */
#define XPKT_OK_INT_STS     (1<<4)

/* packets transmitted to ethernet lost due to late
 * collision or excessive collision
 */
#define XPKT_LOST_INT_STS   (1<<5)

/* packets received into RX FIFO successfully */
#define RPKT_SAV            (1<<6)

/* received packet lost due to RX FIFO full */
#define RPKT_LOST_INT_STS   (1<<7)

#define AHB_ERR             (1<<8)         /* AHB error */
#define PHYSTS_CHG          (1<<9)         /* PHY link status change */
	unsigned int imr;                      /* interrupt mask, 0x4 */
#define RPKT_FINISH_M       (1<<0)         /* interrupt mask of ISR[0] */
#define NORXBUF_M           (1<<1)         /* interrupt mask of ISR[1] */
#define XPKT_FINISH_M       (1<<2)         /* interrupt mask of ISR[2] */
#define NOTXBUF_M           (1<<3)         /* interrupt mask of ISR[3] */
#define XPKT_OK_M           (1<<4)         /* interrupt mask of ISR[4] */
#define XPKT_LOST_M         (1<<5)         /* interrupt mask of ISR[5] */
#define RPKT_SAV_M          (1<<6)         /* interrupt mask of ISR[6] */
#define RPKT_LOST_M         (1<<7)         /* interrupt mask of ISR[7] */
#define AHB_ERR_M           (1<<8)         /* interrupt mask of ISR[8] */
#define PHYSTS_CHG_M        (1<<9)         /* interrupt mask of ISR[9] */

	/* MAC most significant address, 0x8 */
	unsigned int mac_madr;

/* the most significant 2 bytes of MAC address */
#define MAC_MADR_MASK       0xffff

	/* MAC least significant address, 0xc */
	unsigned int mac_ldar;

	/* multicast address hash table 0, 0x10 */
	unsigned int matht0;

	/* multicast address hash table 1, 0x14 */
	unsigned int matht1;

	/* transmit poll demand, 0x18 */
	unsigned int txpd;

	/* receive poll demand, 0x1c */
	unsigned int rxpd;

	/* transmit ring base address, 0x20 */
	unsigned int txr_badr;

	/* receive ring base address, 0x24 */
	unsigned int rxr_badr;

	/* interrupt timer control, 0x28 */
	unsigned int itc;

/* defines the period of TX cycle time */
#define TXINT_TIME_SEL      (1<<15)

#define TXINT_THR_MASK      (1<<14 | 1 < 13 | 1 < 12)
#define TXINT_CNT_MASK      (1<<11 | 1<<10 | 1<<9 | 1<<8)

/* defines the period of RX cycle time */
#define RXINT_TIME_SEL      (1<<7)

#define RXINT_THR_MASK      (1<<6 | 1<<5 | 1<<4)
#define RXINT_CNT_MASK      (1<<3 | 1<<2 | 1<<1 | 1<<0)

	/* automatic polling timer control, 0x2c */
	unsigned int aptc;

/* defines the period of TX poll time */
#define TXPOLL_TIME_SEL     (1<<12)

#define TXPOLL_CNT_MASK     (1<<11 | 1<<10 | 1<<9 | 1<<8)
#define TXPOLL_CNT_SHIFT_BIT    8

/* defines the period of RX poll time */
#define RXPOLL_TIME_SEL     (1<<4)

#define RXPOLL_CNT_MASK     (1<<3 | 1<<2 | 1<<1 | 1<<0)
#define RXPOLL_CNT_SHIFT_BIT    0

	/* DMA burst length and arbitration control, 0x30 */
	unsigned int dblac;

/* enable RX FIFO threshold arbitration */
#define RX_THR_EN           (1<<9)

#define RXFIFO_HTHR_MASK    (1<<8 | 1<<7 | 1<<6)
#define RXFIFO_LTHR_MASK    (1<<5 | 1<<4 | 1<<3)

/* use INCR16 burst command in AHB bus */
#define INCR16_EN           (1<<2)

/* use INCR8 burst command in AHB bus */
#define INCR8_EN            (1<<1)

/* use INCR4 burst command in AHB bus */
#define INCR4_EN            (1<<0)

	unsigned int reserved1[21];            /* 0x34 - 0x84 */
	unsigned int maccr;                    /* MAC control, 0x88 */
#define RX_BROADPKT         (1<<17)        /* receive boradcast packet */

/* receive all multicast packet */
#define RX_MULTIPKT         (1<<16)

#define FULLDUP             (1<<15)        /* full duplex */

/* append CRC to transmitted packet */
#define CRC_APD             (1<<14)

/* not check incoming packet's dest. address */
#define RCV_ALL             (1<<12)

/* store incoming packet even if its length is great than 1518 bytes */
#define RX_FTL              (1<<11)

/* store incoming packet even if its length is less than 64 bytes */
#define RX_RUNT             (1<<10)

/* enable storing incoming packet if the packet passes hash table
 * address filtering and is a multicast packet
 */
#define HT_MULTI_EN         (1<<9)

#define RCV_EN              (1<<8)         /* receiver enable */

/* enable packet reception when transmitting packet in half duplex mode */
#define ENRX_IN_HALFTX      (1<<6)

#define XMT_EN              (1<<5)         /* transmitter enable */

/* disable CRC check when receiving packets */
#define CRC_DIS             (1<<4)

#define LOOP_EN             (1<<3)         /* internal loop-back */

/* software reset, last 64 AHB bus clocks */
#define SW_RST              (1<<2)

#define RDMA_EN             (1<<1)         /* enable receive DMA channel */
#define XDMA_EN             (1<<0)         /* enable transmit DMA channel */
	unsigned int macsr;                    /* MAC status, 0x8c */
#define COL_EXCEED          (1<<11)        /* collision amount exceeds 16 */

/* transmitter detects late collision */
#define LATE_COL            (1<<10)

/* packet transmitted to ethernet lost due to late collision
 * or excessive collision
 */
#define XPKT_LOST           (1<<9)

/* packets transmitted to ethernet successfully */
#define XPKT_OK             (1<<8)

/* receiver detects a runt packet */
#define RUNT_MAC_STS        (1<<7)

/* receiver detects a frame that is too long */
#define FTL_MAC_STS         (1<<6)

#define CRC_ERR_MAC_STS     (1<<5)

/* received packets list due to RX FIFO full */
#define RPKT_LOST           (1<<4)

/* packets received into RX FIFO successfully */
#define RPKT_SAVE           (1<<3)

/* incoming packet dropped due to collision */
#define COL                 (1<<2)

#define MCPU_BROADCAST      (1<<1)
#define MCPU_MULTICAST      (1<<0)
	unsigned int phycr;                    /* PHY control, 0x90 */

/* initialize a write sequence to PHY by setting this bit to 1.
 * This bit would be auto cleared after the write operation is finished.
 */
#define MIIWR               (1<<27)

#define MIIRD               (1<<26)
#define REGAD_MASK          (1<<25 | 1<<24 | 1<<23 | 1<<22 | 1<<21)
#define PHYAD_MASK          (1<<20 | 1<<19 | 1<<18 | 1<<17 | 1<<16)
#define MIIRDATA_MASK       0xffff
	unsigned int phywdata;                 /* PHY write data, 0x94 */
#define MIIWDATA_MASK       0xffff
	unsigned int fcr;                      /* flow control, 0x98 */
#define PAUSE_TIME_MASK     0xffff0000
#define FC_HIGH_MASK        (1<<15 | 1<<14 | 1<<13 | 1<<12)
#define FC_LOW_MASK         (1<<11 | 1<<10 | 1<<9 | 1<<8)
#define RX_PAUSE            (1<<4)         /* receive pause frame */

/* packet transmission is paused due to receive */
#define TXPAUSED            (1<<3)

	/* pause frame */

/* enable flow control threshold mode. */
#define FCTHR_EN            (1<<2)

#define TX_PAUSE            (1<<1)         /* transmit pause frame */
#define FC_EN               (1<<0)         /* flow control mode enable */
	unsigned int bpr;                      /* back pressure, 0x9c */
#define BK_LOW_MASK         (1<<11 | 1<<10 | 1<<9 | 1<<8)
#define BKJAM_LEN_MASK      (1<<7 | 1<<6 | 1<<5 | 1<<4)
#define BK_MODE             (1<<1)         /* back pressure address mode */
#define BK_EN               (1<<0)         /* back pressure mode enable */
	unsigned int reserved2[9];             /* 0xa0 - 0xc0 */
	unsigned int ts;                       /* test seed, 0xc4 */
#define TEST_SEED_MASK      0x3fff
	unsigned int dmafifos;                 /* DMA/FIFO state, 0xc8 */
#define TXD_REQ             (1<<31)        /* TXDMA request */
#define RXD_REQ             (1<<30)        /* RXDMA request */
#define DARB_TXGNT          (1<<29)        /* TXDMA grant */
#define DARB_RXGNT          (1<<28)        /* RXDMA grant */
#define TXFIFO_EMPTY        (1<<27)        /* TX FIFO is empty */
#define RXFIFO_EMPTY        (1<<26)        /* RX FIFO is empty */
#define TXDMA2_SM_MASK      (1<<14 | 1<<13 | 1<<12)
#define TXDMA1_SM_MASK      (1<<11 | 1<<10 | 1<<9 | 1<<8)
#define RXDMA2_SM_MASK      (1<<6 | 1<<5 | 1<<4)
#define RXDMA1_SM_MASK      (1<<3 | 1<<2 | 1<<1 | 1<<0)
	unsigned int tm;                       /* test mode, 0xcc */
#define SINGLE_PKT          (1<<26)        /* single packet mode */

/* automatic polling timer test mode */
#define PTIMER_TEST         (1<<25)

#define ITIMER_TEST         (1<<24)        /* interrupt timer test mode */
#define TEST_SEED_SEL       (1<<22)        /* test seed select */
#define SEED_SEL            (1<<21)        /* seed select */
#define TEST_MODE           (1<<20)        /* transmission test mode */
#define TEST_TIME_MASK      (1<<19 | 1<<18 | 1<<17 | 1<<16 | 1<<15 \
	| 1<<14 | 1<<13 | 1<<12 | 1<<11 | 1<<10)
#define TEST_EXCEL_MASK     (1<<9 | 1<<8 | 1<<7 | 1<<6 | 1<<5)
	unsigned int reserved3;                /* 0xd0 */

	/* TX_MCOL and TX_SCOL counter, 0xd4 */
	unsigned int txmcol_xscol;

#define TX_MCOL_MASK        0xffff0000
#define TX_MCOL_SHIFT_BIT   16
#define TX_SCOL_MASK        0xffff
#define TX_SCOL_SHIFT_BIT   0
	unsigned int rpf_aep;                  /* RPF and AEP counter, 0xd8 */
#define RPF_MASK        0xffff0000
#define RPF_SHIFT_BIT       16
#define AEP_MASK        0xffff
#define AEP_SHIFT_BIT       0
	unsigned int xm_pg;                    /* XM and PG counter, 0xdc */
#define XM_MASK         0xffff0000
#define XM_SHIFT_BIT        16
#define PG_MASK         0xffff
#define PG_SHIFT_BIT        0

	/* RUNT_CNT and TLCC counter, 0xe0 */
	unsigned int runtcnt_tlcc;

#define RUNT_CNT_MASK       0xffff0000
#define RUNT_CNT_SHIFT_BIT  16
#define TLCC_MASK       0xffff
#define TLCC_SHIFT_BIT      0

	/* CRCER_CNT and FTL_CNT counter, 0xe4 */
	unsigned int crcercnt_ftlcnt;

#define CRCER_CNT_MASK      0xffff0000
#define CRCER_CNT_SHIFT_BIT 16
#define FTL_CNT_MASK        0xffff
#define FTL_CNT_SHIFT_BIT   0
	unsigned int rlc_rcc;                  /* RLC and RCC counter, 0xe8 */
#define RLC_MASK        0xffff0000
#define RLC_SHIFT_BIT       16
#define RCC_MASK        0xffff
#define RCC_SHIFT_BIT       0
	unsigned int broc;                     /* BROC counter, 0xec */
	unsigned int mulca;                    /* MULCA counter, 0xf0 */
	unsigned int rp;                       /* RP counter, 0xf4 */
	unsigned int xp;                       /* XP counter, 0xf8 */
};

#define ISR_REG_OFFSET                             0x00
#define IMR_REG_OFFSET                             0x04
#define MAC_MADR_REG_OFFSET                        0x08
#define MAC_LADR_REG_OFFSET                        0x0C
#define MATH0_REG_OFFSET                           0x10
#define MATH1_REG_OFFSET                           0x14
#define TXPD_REG_OFFSET                            0x18
#define RXPD_REG_OFFSET                            0x1C
#define TXR_BADR_REG_OFFSET                        0x20
#define RXR_BADR_REG_OFFSET                        0x24
#define ITC_REG_OFFSET                             0x28
#define APTC_REG_OFFSET                            0x2C
#define DBLAC_REG_OFFSET                           0x30
#define MACCR_REG_OFFSET                           0x88
#define MACSR_REG_OFFSET                           0x8C
#define PHYCR_REG_OFFSET                           0x90
#define PHYWDATA_REG_OFFSET                        0x94
#define FCR_REG_OFFSET                             0x98
#define BPR_REG_OFFSET                             0x9C
#define TS_REG_OFFSET                              0xC4
#define DMAFIFOS_REG_OFFSET                        0xC8
#define TM_REG_OFFSET                              0xCC
#define TX_MCOL_TX_SCOL_REG_OFFSET                 0xD4
#define RPF_AEP_REG_OFFSET                         0xD8
#define XM_PG_REG_OFFSET                           0xDC
#define RUNT_CNT_TLCC_REG_OFFSET                   0xE0
#define CRCER_CNT_FTL_CNT_REG_OFFSET               0xE4
#define RLC_RCC_REG_OFFSET                         0xE8
#define BROC_REG_OFFSET                            0xEC
#define MULCA_REG_OFFSET                           0xF0
#define RP_REG_OFFSET                              0xF4
#define XP_REG_OFFSET                              0xF8
#define PHY_CNTL_REG                               0x00
#define PHY_STATUS_REG                             0x01
#define PHY_ID_REG1                                0x02
#define PHY_ID_REG2                                0x03
#define PHY_ANA_REG                                0x04
#define PHY_ANLPAR_REG                             0x05
#define PHY_ANE_REG                                0x06
#define PHY_ECNTL_REG1                             0x10
#define PHY_QPDS_REG                               0x11
#define PHY_10BOP_REG                              0x12
#define PHY_ECNTL_REG2                             0x13
#define FTMAC100_REG_PHY_WRITE                     0x08000000
#define FTMAC100_REG_PHY_READ                      0x04000000

/* PHY Status register */
#define AN_COMPLETE                                0x0020

#define LINK_STATUS                                0x0004

struct moxart_mac_priv_t {
	void __iomem *base;
	void __iomem *flash_base;

	/* Tx descriptor physical base address */
	unsigned int phy_tx_desc_baseaddr;

	/* Tx descriptor virtual base address */
	struct tx_desc_t *virt_tx_desc_baseaddr;

	/* Rx descriptor physical base address */
	unsigned int phy_rx_desc_baseaddr;

	/* Rx descriptor virtual base address */
	struct rx_desc_t *virt_rx_desc_baseaddr;

	/* Tx buffer physical base address */
	unsigned int phy_tx_buf_baseaddr;

	/* Tx buffer virtual base address */
	unsigned char *virt_tx_buf_baseaddr;

	/* Rx buffer physical base address */
	unsigned int phy_rx_buf_baseaddr;

	/* Rx buffer virtual base address */
	unsigned char *virt_rx_buf_baseaddr;

	/* Tx descriptor now first used index */
	int tx_desc_now;

	/* Rx descriptor now first used index */
	int rx_desc_now;

	/* OS about the ethernet statistics */
	struct net_device_stats stats;

	spinlock_t txlock;
	spinlock_t rxlock;

	/* store the maccr control register value */
	unsigned int maccr;

	struct work_struct rqueue;
};

#if TX_BUF_SIZE >= TXBUF_SIZE_MAX
#error Moxa CPU ethernet device driver Tx buffer size too large !
#endif
#if RX_BUF_SIZE >= RXBUF_SIZE_MAX
#error Moxa CPU ethernet device driver Rx buffer size too large !
#endif

#endif /* MOXACPU_MAC_H */
