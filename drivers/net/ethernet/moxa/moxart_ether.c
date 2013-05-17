/* MOXART Ethernet (RTL8201CP) device driver (based on MOXA sources)
 * Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "moxart_ether.h"

static inline unsigned long moxart_emac_read(struct net_device *dev,
	unsigned int reg)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	return readl(priv->base + reg);
}

static inline void moxart_emac_write(struct net_device *dev,
	unsigned int reg, unsigned long value)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	writel(value, priv->base + reg);
}

/* Program the hardware MAC address from dev->dev_addr */
static void moxart_update_mac_address(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	moxart_emac_write(dev, MAC_MADR_REG_OFFSET,
		((dev->dev_addr[0] << 8) | (dev->dev_addr[1])));
	moxart_emac_write(dev, MAC_MADR_REG_OFFSET + 4,
		((dev->dev_addr[2] << 24) | (dev->dev_addr[3] << 16) |
		  (dev->dev_addr[4] << 8) | (dev->dev_addr[5])));

	netdev_dbg(dev, "%s: base=%p MAC=%x%x\n",
		__func__, priv->base,
		readl(priv->base + MAC_MADR_REG_OFFSET),
		readl(priv->base + MAC_MADR_REG_OFFSET + 4));
}

static int moxart_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;


	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	moxart_update_mac_address(dev);

	netdev_dbg(dev, "%s: base_addr=%x\n",
		__func__, (unsigned int) dev->base_addr);
	return 0;
}

static void moxart_mac_free_memory(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	if (priv->virt_tx_desc_baseaddr)
		dma_free_coherent(NULL, sizeof(struct tx_desc_t)*TX_DESC_NUM,
			priv->virt_tx_desc_baseaddr,
			priv->phy_tx_desc_baseaddr);
	if (priv->virt_rx_desc_baseaddr)
		dma_free_coherent(NULL, sizeof(struct rx_desc_t)*RX_DESC_NUM,
			priv->virt_rx_desc_baseaddr,
			priv->phy_rx_desc_baseaddr);
	if (priv->virt_tx_buf_baseaddr)
		dma_free_coherent(NULL, TX_BUF_SIZE*TX_DESC_NUM,
			priv->virt_tx_buf_baseaddr,
			priv->phy_tx_buf_baseaddr);
	if (priv->virt_rx_buf_baseaddr)
		dma_free_coherent(NULL, RX_BUF_SIZE*RX_DESC_NUM,
			priv->virt_rx_buf_baseaddr,
			priv->phy_rx_buf_baseaddr);
}

static void moxart_mac_reset(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	writel(SW_RST, priv->base + MACCR_REG_OFFSET);
	while (readl(priv->base + MACCR_REG_OFFSET) & SW_RST)
		mdelay(10);

	writel(0, priv->base + IMR_REG_OFFSET);

	priv->maccr = RX_BROADPKT | FULLDUP | CRC_APD | RX_FTL;

	netdev_dbg(dev, "%s: base=%p\n",
		__func__, priv->base);
}

static void moxart_mac_enable(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	writel(0x00001010, priv->base + ITC_REG_OFFSET);
	writel(0x00000001, priv->base + APTC_REG_OFFSET);
	writel(0x00000390, priv->base + DBLAC_REG_OFFSET);

	/* use NORXBUF_M to test interrupt */
	/* writel(RPKT_FINISH_M | NORXBUF_M | AHB_ERR_M,
		priv->base + IMR_REG_OFFSET);
	*/

	writel(RPKT_FINISH_M, priv->base + IMR_REG_OFFSET);
	priv->maccr |= (RCV_EN | XMT_EN | RDMA_EN | XDMA_EN);
	writel(priv->maccr, priv->base + MACCR_REG_OFFSET);

	netdev_dbg(dev, "%s: base=%p\n",
		__func__, priv->base);
}

static void moxart_mac_setup_desc_ring(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	struct tx_desc_t *txdesc;
	struct rx_desc_t *rxdesc;
	unsigned char *virtbuf;
	unsigned int phybuf;
	int i;

	virtbuf = priv->virt_tx_buf_baseaddr;
	phybuf = priv->phy_tx_buf_baseaddr;
	for (i = 0; i < TX_DESC_NUM; i++,
		virtbuf += TX_BUF_SIZE, phybuf += TX_BUF_SIZE) {
		txdesc = &priv->virt_tx_desc_baseaddr[i];
		memset(txdesc, 0, sizeof(struct tx_desc_t));
		txdesc->txdes2.phy_tx_buf_baseaddr = phybuf;
		txdesc->txdes2.virt_tx_buf_baseaddr = virtbuf;
	}
	priv->virt_tx_desc_baseaddr[TX_DESC_NUM - 1].txdes1.ubit.edotr = 1;

	virtbuf = priv->virt_rx_buf_baseaddr;
	phybuf = priv->phy_rx_buf_baseaddr;
	for (i = 0; i < RX_DESC_NUM; i++,
		virtbuf += RX_BUF_SIZE, phybuf += RX_BUF_SIZE) {
		rxdesc = &priv->virt_rx_desc_baseaddr[i];
		memset(rxdesc, 0, sizeof(struct rx_desc_t));
		rxdesc->rxdes0.ubit.rx_dma_own = 1;
		rxdesc->rxdes1.ubit.rx_buf_size = RX_BUF_SIZE;
		rxdesc->rxdes2.phy_rx_buf_baseaddr = phybuf;
		rxdesc->rxdes2.virt_rx_buf_baseaddr = virtbuf;
	}
	priv->virt_rx_desc_baseaddr[RX_DESC_NUM - 1].rxdes1.ubit.edorr = 1;
	/* pr_debug("First Rx desc des0=0x%x, des1=%x\n",
		priv->virt_rx_desc_baseaddr[0].rxdes0.ui,
		priv->virt_rx_desc_baseaddr[0].rxdes1.ui);
	*/

	priv->tx_desc_now = priv->rx_desc_now = 0;

	/* reset the MAC controler Tx/Rx desciptor base address */
	writel(priv->phy_tx_desc_baseaddr, priv->base + TXR_BADR_REG_OFFSET);
	writel(priv->phy_rx_desc_baseaddr, priv->base + RXR_BADR_REG_OFFSET);

	netdev_dbg(dev,
		"%s: base=%p\n"
		"Tx/Rx desc phy=0x%x,0x%x, virt=0x%x,0x%x\n"
		"set Tx desc base address=0x%x, Rx=0x%x\n",
		__func__, priv->base, priv->phy_tx_desc_baseaddr,
		priv->phy_rx_desc_baseaddr,
		(unsigned int)priv->virt_tx_desc_baseaddr,
		(unsigned int)priv->virt_rx_desc_baseaddr,
		readl(priv->base + TXR_BADR_REG_OFFSET),
		readl(priv->base + RXR_BADR_REG_OFFSET));
}

static int moxart_mac_open(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	if (!is_valid_ether_addr(dev->dev_addr))
		return -EADDRNOTAVAIL;

	spin_lock_irq(&priv->txlock);
	moxart_mac_reset(dev);
	moxart_update_mac_address(dev);
	moxart_mac_setup_desc_ring(dev);
	moxart_mac_enable(dev);
	spin_unlock_irq(&priv->txlock);
	netif_start_queue(dev);

	netdev_dbg(dev, "%s: IMR=0x%x, MACCR=0x%x\n",
		__func__,
		readl(priv->base + IMR_REG_OFFSET),
		readl(priv->base + MACCR_REG_OFFSET));
	return 0;
}

static int moxart_mac_stop(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	spin_lock_irq(&priv->txlock);

	/* disable all interrupt */
	writel(0, priv->base + IMR_REG_OFFSET);

	/* disable all function */
	writel(0, priv->base + MACCR_REG_OFFSET);

	spin_unlock_irq(&priv->txlock);

	netdev_dbg(dev, "%s: finished moxart_mac_stop\n", __func__);
	return 0;
}

static void moxart_mac_recv(struct work_struct *ptr)
{
	struct net_device *dev = (struct net_device *) ptr;
	struct moxart_mac_priv_t *priv = netdev_priv((struct net_device *)ptr);
	struct rx_desc_t *rxdesc;
	struct sk_buff *skb;
	unsigned int ui, len;
	int rxnow = priv->rx_desc_now;
	int loops = RX_DESC_NUM;

	/*netdev_dbg(dev, "%s\n", __func__);
	netdev_dbg(dev,
		"%s: rx_d=%d, phy_d=0x%x, vrt=0x%x, rx_buf_d=0x%x, vrt=0x%x\n",
		__func__, priv->rx_desc_now,
		priv->phy_rx_desc_baseaddr+
		(priv->rx_desc_now*sizeof(rx_desc_t)),
		(unsigned int)&priv->virt_rx_desc_baseaddr[priv->rx_desc_now],
		priv->virt_rx_desc_baseaddr[priv->rx_desc_now]
		.rxdes2.phy_rx_buf_baseaddr,
		(unsigned int)priv->virt_rx_desc_baseaddr[priv->rx_desc_now]
		.rxdes2.virt_rx_buf_baseaddr);
	netdev_dbg(dev, "%s: Now Rx desc des0=0x%x, des1=0x%x\n", __func__,
		priv->virt_rx_desc_baseaddr[priv->rx_desc_now].rxdes0.ui,
		priv->virt_rx_desc_baseaddr[priv->rx_desc_now].rxdes1.ui);
	*/

repeat_recv:
	rxdesc = &priv->virt_rx_desc_baseaddr[rxnow];
	ui = rxdesc->rxdes0.ui;

	/* if ( rxdesc->rxdes0.ubit.RxDMAOwn ) { */
	if (ui & RXDMA_OWN)
		return;

	if (ui & (RX_ERR | CRC_ERR | FTL | RUNT | RX_ODD_NB)) {
		netdev_err(dev, "%s: packet error!\n", __func__);
		priv->stats.rx_dropped++;
		priv->stats.rx_errors++;
		goto recv_finish;
	}

	len = ui & RFL_MASK;

	if (len > RX_BUF_SIZE)
		len = RX_BUF_SIZE;

	skb = dev_alloc_skb(len + 2);
	if (skb == NULL) {
		netdev_err(dev, "%s: memory alloc failed!\n", __func__);
		priv->stats.rx_dropped++;
		goto recv_finish;
	}
	skb_reserve(skb, 2);
	skb->dev = dev;

	memcpy(skb_put(skb, len), rxdesc->rxdes2.virt_rx_buf_baseaddr, len);
	netif_rx(skb);

	/* pr_debug("MOXART Ethernet: receive data pointer = 0x%x\n",
		(unsigned long)data);
	*/

	skb->protocol = eth_type_trans(skb, dev);
	dev->last_rx = jiffies;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;
	if (ui & MULTICAST_RXDES0)
		priv->stats.multicast++;

recv_finish:
	rxdesc->rxdes0.ui = RXDMA_OWN;
	rxnow++;
	rxnow &= RX_DESC_NUM_MASK;
	priv->rx_desc_now = rxnow;
	if (loops-- > 0)
		goto repeat_recv;
}

static irqreturn_t moxart_mac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	unsigned int ists = readl(priv->base + ISR_REG_OFFSET);

	/* pr_debug("MOXART Ethernet: moxart_mac_interrupt\n"); */

	if (ists & RPKT_FINISH) {
		moxart_mac_recv((void *) dev);
	} else {
#ifdef MOXART_MAC_DEBUG
		if (ists & NORXBUF) {
			netdev_dbg(dev, "%s: NORXBUF interrupt\n", __func__);
			writel(readl(priv->base + IMR_REG_OFFSET) & ~NORXBUF_M,
				priv->base + IMR_REG_OFFSET);
		}
		if (ists & AHB_ERR)
			netdev_dbg(dev, "%s: AHB_ERR interrupt.\n", __func__);
#endif
	}
	return IRQ_HANDLED;
}

static int moxart_mac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	struct tx_desc_t *txdesc;
	int len;
	int txnow = priv->tx_desc_now;

	/* pr_debug("MOXART Ethernet: moxart_mac_hard_start_xmit\n"); */

	spin_lock_irq(&priv->txlock);
	txdesc = &priv->virt_tx_desc_baseaddr[txnow];
	if (txdesc->txdes0.ubit.tx_dma_own) {
		netdev_dbg(dev, "%s: no TX space for packet!\n", __func__);
		priv->stats.tx_dropped++;
		goto xmit_final;
	}

	/*len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	len = len > TX_BUF_SIZE ? TX_BUF_SIZE : len;
	*/

	len = skb->len > TX_BUF_SIZE ? TX_BUF_SIZE : skb->len;
	memcpy(txdesc->txdes2.virt_tx_buf_baseaddr, skb->data, len);

	if (skb->len < ETH_ZLEN) {
		memset(&txdesc->txdes2.virt_tx_buf_baseaddr[skb->len],
			0, ETH_ZLEN - skb->len);
		len = ETH_ZLEN;
	}

	txdesc->txdes1.ubit.lts = 1;
	txdesc->txdes1.ubit.fts = 1;
	txdesc->txdes1.ubit.tx2_fic = 0;
	txdesc->txdes1.ubit.tx_ic = 0;
	txdesc->txdes1.ubit.tx_buf_size = len;
	txdesc->txdes0.ui = TXDMA_OWN;

	netdev_dbg(dev, "%s: transmit data pointer = 0x%x\n",
		__func__, (unsigned int)skb->data);

	/* start to send packet */
	writel(0xffffffff, priv->base + TXPD_REG_OFFSET);

	/* netdev_dbg(dev, "%s: tx_d_now=%d, adr=0x%x, des0=0x%x, des1=0x%x\n",
		__func__, priv->tx_desc_now,
		(unsigned int)&priv->virt_tx_desc_baseaddr[priv->tx_desc_now],
		txdesc->txdes0.ui, txdesc->txdes1.ui);
	netdev_dbg(dev, "%s: Buffer phy address=0x%x, virt=0x%x\n",
		__func__, txdesc->txdes2.phy_tx_buf_baseaddr,
		(unsigned int)txdesc->txdes2.virt_tx_buf_baseaddr);
	netdev_dbg(dev, "%s: tx_desc_now-1=%d, address=0x%x, des0=0x%x\n",
		__func__, (priv->tx_desc_now-1)&TX_DESC_NUM_MASK,
		(unsigned int)&priv->virt_tx_desc_baseaddr[(priv->tx_desc_now-1)
		&TX_DESC_NUM_MASK],
		priv->virt_tx_desc_baseaddr[(priv->tx_desc_now-1)
		&TX_DESC_NUM_MASK].txdes0.ui);
	 */

	txnow++;
	txnow &= TX_DESC_NUM_MASK;
	priv->tx_desc_now = txnow;
	dev->trans_start = jiffies;
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;

xmit_final:
	spin_unlock_irq(&priv->txlock);
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static struct net_device_stats *moxart_mac_get_stats(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	int desc = priv->rx_desc_now;

	netdev_dbg(dev, "%s:\n", __func__);
	netdev_dbg(dev,
		"%s: rx_desc_now=%d, desc phy=0x%x, virt=0x%x, buf phy=0x%x\n"
		"virt=0x%x Now Rx desc des0=0x%x, des1=0x%x\n", __func__, desc,
		priv->phy_rx_desc_baseaddr+(desc*sizeof(struct rx_desc_t)),
		(unsigned int)&priv->virt_rx_desc_baseaddr[desc],
		priv->virt_rx_desc_baseaddr[desc].rxdes2.phy_rx_buf_baseaddr,
		(unsigned int)priv->virt_rx_desc_baseaddr[desc]
	    .rxdes2.virt_rx_buf_baseaddr,
		priv->virt_rx_desc_baseaddr[desc].rxdes0.ui,
		priv->virt_rx_desc_baseaddr[desc].rxdes1.ui);
	desc++;
	desc &= RX_DESC_NUM_MASK;
	netdev_dbg(dev,
		"%s: rx_desc_now=%d, desc phy=0x%x, virt=0x%x, buf phy=0x%x\n"
		"virt=0x%x Now Rx desc des0=0x%x, des1=0x%x (desc++)\n",
		__func__, desc, priv->phy_rx_desc_baseaddr +
		(desc*sizeof(struct rx_desc_t)),
		(unsigned int)&priv->virt_rx_desc_baseaddr[desc],
		priv->virt_rx_desc_baseaddr[desc].rxdes2.phy_rx_buf_baseaddr,
		(unsigned int)priv->virt_rx_desc_baseaddr[desc]
	    .rxdes2.virt_rx_buf_baseaddr,
		priv->virt_rx_desc_baseaddr[desc].rxdes0.ui,
		priv->virt_rx_desc_baseaddr[desc].rxdes1.ui);

	/*don't inl(dev->base_addr.. here or suffer the NULL deref */
	netdev_dbg(dev,
		"%s: TX_MCOL_TX_SCOL=0x%x RPF_AEP=0x%x XM_PG=0x%x\n"
		" RUNT_CNT_TLCC=0x%x CRCER_CNT_FTL_CNT=0x%x RLC_RCC=0x%x\n"
		" BROC=0x%x MUCLA=0x%x RP=0x%x XP=0x%x\n", __func__,
		readl(priv->base + TX_MCOL_TX_SCOL_REG_OFFSET),
		readl(priv->base + RPF_AEP_REG_OFFSET),
			readl(priv->base + XM_PG_REG_OFFSET),
		readl(priv->base + RUNT_CNT_TLCC_REG_OFFSET),
		readl(priv->base + CRCER_CNT_FTL_CNT_REG_OFFSET),
		readl(priv->base + RLC_RCC_REG_OFFSET),
			readl(priv->base + BROC_REG_OFFSET),
		readl(priv->base + MULCA_REG_OFFSET),
			readl(priv->base + RP_REG_OFFSET),
		readl(priv->base + XP_REG_OFFSET));

	return &priv->stats;
}

#ifdef HAVE_MULTICAST

static int crc32(char *s, int length)
{
	int per_byte;
	int per_bit;

	/* crc polynomial */
	const unsigned long poly = 0xedb88320;

	/* crc value - preinitialized to all 1's */
	unsigned long crc_value = 0xffffffff;

	for (per_byte = 0; per_byte < length; per_byte++) {
		unsigned char c;
		c = *(s++);
		for (per_bit = 0; per_bit < 8; per_bit++) {
			crc_value = (crc_value >> 1) ^
				(((crc_value ^ c) & 0x01) ? poly : 0);
			c >>= 1;
		}
	}
	return crc_value;
}

static void moxart_mac_setmulticast(void __iomem base,
	int count, struct dev_mc_list *addrs)
{
	struct dev_mc_list *cur_addr;
	int crc_val;

	for (cur_addr = addrs; cur_addr != NULL; cur_addr = cur_addr->next) {
		if (!(*cur_addr->dmi_addr & 1))
			continue;
		crc_val = crc32(cur_addr->dmi_addr, 6);
		crc_val = (crc_val >> 26) & 0x3f; /* MSB 6 bit */
		if (crc_val >= 32) {
			writel(readl(base + MATH1_REG_OFFSET) |
				(1UL << (crc_val - 32)),
				base + MATH1_REG_OFFSET);
		} else {
			writel(readl(base + MATH0_REG_OFFSET) |
				(1UL << crc_val),
				base + MATH0_REG_OFFSET);
		}
	}
}

static void moxart_mac_set_multicast_list(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);

	spin_lock_irq(&priv->txlock);

	(dev->flags & IFF_PROMISC) ? (priv->maccr |= RCV_ALL) :
			(priv->maccr &= ~RCV_ALL);
	(dev->flags & IFF_ALLMULTI) ? (priv->maccr |= RX_MULTIPKT) :
			(priv->maccr &= ~RX_MULTIPKT);

	if (dev->mc_count) {
		priv->maccr |= HT_MULTI_EN;
		moxart_mac_setmulticast(priv->base,
			dev->mc_count, dev->mc_list);
	} else {
		priv->maccr &= ~HT_MULTI_EN;
	}

	writel(priv->maccr, priv->base + MACCR_REG_OFFSET);

	spin_unlock_irq(&priv->txlock);
}

#endif	/* HAVE_MULTICAST */

static struct net_device_ops moxart_netdev_ops = {
	.ndo_open					= moxart_mac_open,
	.ndo_stop					= moxart_mac_stop,
	.ndo_start_xmit				= moxart_mac_start_xmit,
	.ndo_get_stats				= moxart_mac_get_stats,
#ifdef HAVE_MULTICAST
	.ndo_set_multicast_list		= moxart_mac_set_multicast_list,
#endif
	.ndo_set_mac_address		= moxart_set_mac_address,
/*	.ndo_do_ioctl				= moxart_mac_iotcl, */
	.ndo_validate_addr			= eth_validate_addr,
	.ndo_change_mtu				= eth_change_mtu,
/*	.ndo_tx_timeout				= moxart_mac_tx_timeout,
	.ndo_init					= moxart_mac_init,
*/
};

/* Get MAC address stored in flash memory and write it to net_device */
static void moxart_get_mac_address(struct net_device *dev)
{
	struct moxart_mac_priv_t *priv = netdev_priv(dev);
	int i;

	netdev_dbg(dev,
		"%s base_addr=%lx base=%p flash_base=%p\n",
		__func__, dev->base_addr, priv->base,
		priv->flash_base);

	for (i = 0; i <= 5; i++)
		dev->dev_addr[i] = readb(priv->flash_base + i);
}

static int moxart_mac_probe(struct platform_device *pdev)
{
	struct device *p_dev = &pdev->dev;
	struct device_node *node = p_dev->of_node;
	struct net_device *dev;
	struct moxart_mac_priv_t *priv;
	struct resource res_mac, res_flash;
	int err;
	unsigned int irq;

	dev = alloc_etherdev(sizeof(struct moxart_mac_priv_t));
	if (!dev)
		return -ENOMEM;

	err = of_address_to_resource(node, 0, &res_mac);
	if (err) {
		dev_err(p_dev, "could not get mac base resource\n");
		goto init_fail;
	}
	err = of_address_to_resource(node, 1, &res_flash);
	if (err) {
		dev_err(p_dev, "could not get flash base resource\n");
		goto init_fail;
	}

	irq = irq_of_parse_and_map(node, 0);

	priv = netdev_priv(dev);

	dev->base_addr = res_mac.start;
	priv->base = devm_ioremap_resource(p_dev, &res_mac);
	if (IS_ERR(priv->base)) {
		dev_err(p_dev, "%s: devm_ioremap_resource res_mac failed\n",
			__func__);
		goto init_fail;
	}
	/*  use ioremap here instead of devm_ioremap_resource
		physmap_of will request the memory region first
		and doing it here again would fail
		if flash partition 0 (bootloader) was omitted from DT
		then this driver could request the region exclusively
	*/
	priv->flash_base = ioremap(res_flash.start, resource_size(&res_flash));
	if (IS_ERR(priv->flash_base)) {
		dev_err(p_dev, "%s: devm_ioremap_resource res_flash failed\n",
			__func__);
		goto init_fail;
	}

	/* initialize the private variable to zero */
	/* memset((void *) priv, 0, sizeof(struct moxart_mac_priv_t)); */

	spin_lock_init(&priv->txlock);
	priv->virt_tx_desc_baseaddr = (struct tx_desc_t *)
		dma_alloc_coherent(NULL, sizeof(struct tx_desc_t) * TX_DESC_NUM,
		(dma_addr_t *)&priv->phy_tx_desc_baseaddr,
		GFP_DMA | GFP_KERNEL);
	if (priv->virt_tx_desc_baseaddr == NULL ||
		(priv->phy_tx_desc_baseaddr & 0x0f)) {
		netdev_err(dev, "TX descriptor alloc failed!\n");
		goto init_fail;
	}
	priv->virt_rx_desc_baseaddr = (struct rx_desc_t *)
		dma_alloc_coherent(NULL, sizeof(struct rx_desc_t) * RX_DESC_NUM,
		(dma_addr_t *)&priv->phy_rx_desc_baseaddr,
		GFP_DMA | GFP_KERNEL);
	if (priv->virt_rx_desc_baseaddr == NULL ||
		(priv->phy_rx_desc_baseaddr & 0x0f)) {
		netdev_err(dev, "RX descriptor alloc failed!\n");
		goto init_fail;
	}
	priv->virt_tx_buf_baseaddr = (unsigned char *)
		dma_alloc_coherent(NULL, TX_BUF_SIZE * TX_DESC_NUM,
		(dma_addr_t *)&priv->phy_tx_buf_baseaddr, GFP_DMA | GFP_KERNEL);
	if (priv->virt_tx_buf_baseaddr == NULL ||
		(priv->phy_tx_buf_baseaddr & 0x03)) {
		netdev_err(dev, "TX buffer alloc failed!\n");
		goto init_fail;
	}
	priv->virt_rx_buf_baseaddr = (unsigned char *)
		dma_alloc_coherent(NULL, RX_BUF_SIZE * RX_DESC_NUM,
		(dma_addr_t *)&priv->phy_rx_buf_baseaddr, GFP_DMA | GFP_KERNEL);
	if (priv->virt_rx_buf_baseaddr == NULL ||
		(priv->phy_rx_buf_baseaddr & 0x03)) {
		netdev_err(dev, "RX buffer alloc failed!\n");
		goto init_fail;
	}
	platform_set_drvdata(pdev, dev);

	ether_setup(dev);
	dev->netdev_ops = &moxart_netdev_ops;

	SET_NETDEV_DEV(dev, &pdev->dev);

	moxart_get_mac_address(dev);
	moxart_update_mac_address(dev);

	if (register_netdev(dev)) {
		free_netdev(dev);
		goto init_fail;
	}

	dev->irq = irq;

	if (devm_request_irq(p_dev, irq, moxart_mac_interrupt,
		IRQF_DISABLED, pdev->name, dev)) {
		netdev_info(dev, "%s: devm_request_irq failed\n", __func__);
		free_netdev(dev);
		return -EBUSY;
	}

	netdev_info(dev, "%s: IRQ=%d address=%02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__, dev->irq,
		dev->dev_addr[0], dev->dev_addr[1],	dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4],	dev->dev_addr[5]);
	return 0;

init_fail:
	netdev_info(dev, "%s: init_fail!\n", __func__);
	moxart_mac_free_memory(dev);
	return -ENOMEM;
}

static int moxart_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	unregister_netdev(dev);
	free_irq(dev->irq, dev);
	moxart_mac_free_memory(dev);
	platform_set_drvdata(pdev, NULL);
	free_netdev(dev);
	return 0;
}

static const struct of_device_id moxart_mac_match[] = {
	{ .compatible = "moxa,moxart-mac0" },
	{ .compatible = "moxa,moxart-mac1" },
	{ }
};

struct __initdata platform_driver moxart_mac_driver = {
	.probe       = moxart_mac_probe,
	.remove      = moxart_remove,
	.driver      = {
		.name			= "moxart-ethernet",
		.owner			= THIS_MODULE,
		.of_match_table	= moxart_mac_match,
	},
/*	.suspend     = NULL,
	.resume      = NULL,
*/
};

static int __init moxart_mac_init(void)
{
	return platform_driver_register(&moxart_mac_driver);
}

static void __exit moxart_mac_exit(void)
{
	platform_driver_unregister(&moxart_mac_driver);
}

module_init(moxart_mac_init)
module_exit(moxart_mac_exit)

MODULE_ALIAS("platform:moxart-ethernet");
MODULE_DESCRIPTION("MOXART RTL8201CP Ethernet driver");
MODULE_VERSION("1.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Jensen <jonas.jensen@gmail.com>");

