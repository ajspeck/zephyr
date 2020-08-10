/* Stellaris Ethernet Controller
 *
 * Copyright (c) 2018 Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xmc_xmc4xxx_ethernet

#define LOG_MODULE_NAME eth_xmc4xxx
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <device.h>
#include <soc.h>
#include <ethernet/eth_stats.h>
#include "eth_xmc4xxx_priv.h"
#include <xmc_eth_mac.h>
#include <xmc_eth_phy.h>
#include <xmc_gpio.h>

#define ETH_LWIP_0_CRS_DV  XMC_GPIO_PORT15, 9U
#define ETH_LWIP_0_RXER  XMC_GPIO_PORT2, 4U
#define ETH_LWIP_0_RXD0  XMC_GPIO_PORT2, 2U
#define ETH_LWIP_0_RXD1  XMC_GPIO_PORT2, 3U
#define ETH_LWIP_0_TXEN  XMC_GPIO_PORT2, 5U
#define ETH_LWIP_0_TXD0  XMC_GPIO_PORT2, 8U
#define ETH_LWIP_0_TXD1  XMC_GPIO_PORT2, 9U
#define ETH_LWIP_0_RMII_CLK  XMC_GPIO_PORT15, 8U
#define ETH_LWIP_0_MDC  XMC_GPIO_PORT2, 7U
#define ETH_LWIP_0_MDIO  XMC_GPIO_PORT2, 0U
#define ETH_LWIP_0_PIN_LIST_SIZE 10U
#define ETH_LWIP_0_PHY_ADDR   (0)

#define ETH_LWIP_0_NUM_RX_BUF (8U)
#define ETH_LWIP_0_NUM_TX_BUF (8U)
#define ETH_LWIP_PHY_MAX_RETRIES  0xfffffU
#if defined(__GNUC__)
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][ETH_RX_BUF_SIZE] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][ETH_TX_BUF_SIZE] __attribute__((section ("ETH_RAM")));
#else
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF];
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF];
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][ETH_RX_BUF_SIZE];
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][ETH_TX_BUF_SIZE];
#endif
const XMC_ETH_PHY_CONFIG_t eth_phy_config =
{
    .interface = XMC_ETH_LINK_INTERFACE_RMII,
    .enable_auto_negotiate = true
};

static XMC_ETH_MAC_t eth_mac =
{
    .regs = ETH0,
    .rx_desc = ETH_LWIP_0_rx_desc,
    .tx_desc = ETH_LWIP_0_tx_desc,
    .rx_buf = &ETH_LWIP_0_rx_buf[0][0],
    .tx_buf = &ETH_LWIP_0_tx_buf[0][0],
    .num_rx_buf = ETH_LWIP_0_NUM_RX_BUF,
    .num_tx_buf = ETH_LWIP_0_NUM_TX_BUF
};

static void eth_xmc4xxx_assign_mac(struct device *dev)
{
	uint8_t mac_addr[6] = DT_INST_PROP(0, local_mac_address);
	uint64_t value = 0x0;
	value |= mac_addr[0];
	value |= mac_addr[1] << 8;
	value |= mac_addr[2] << 16;
	value |= mac_addr[3] << 24;
	value |= ((uint64_t)(mac_addr[4])) << 32;
	value |= ((uint64_t)(mac_addr[5])) << 40;
  	eth_mac.address = value;

	XMC_ETH_MAC_SetAddress(&eth_mac, value);
}

/* Get RX frame size */
bool XMC_ETH_MAC_ClearLargeFrames(XMC_ETH_MAC_t *const eth_mac)
{
  uint32_t status;

  status = eth_mac->rx_desc[eth_mac->rx_index].status;

  if (status & ETH_MAC_DMA_RDES0_OWN)
  {
    /* Owned by DMA */
    return false;
  }
  if ((status & ETH_MAC_DMA_RDES0_ES) != 0U)
  {
	  return false;
  }
  if ((status & ETH_MAC_DMA_RDES0_FS) == 0U)
  {
	  return false;
  }
  int i = 0;
  while (((status & ETH_MAC_DMA_RDES0_LS) == 0U) && (i<ETH_LWIP_0_NUM_RX_BUF))
  {
		if (status & ETH_MAC_DMA_RDES0_OWN)
		{
			/* Owned by DMA */
			return false;
		}
		i++;
		eth_mac->rx_desc[eth_mac->rx_index].status = (uint32_t)ETH_MAC_DMA_RDES0_OWN;
		XMC_ETH_MAC_ReturnRxDescriptor(eth_mac);
		status = eth_mac->rx_desc[eth_mac->rx_index].status;  
  }
  if ((!(status & ETH_MAC_DMA_RDES0_OWN)) && ((status & ETH_MAC_DMA_RDES0_LS) != 0U))
  {
		eth_mac->rx_desc[eth_mac->rx_index].status = (uint32_t)ETH_MAC_DMA_RDES0_OWN;
	  	XMC_ETH_MAC_ReturnRxDescriptor(eth_mac);
  }
  return true;
}

static struct net_if *get_iface(struct eth_xmc4xxx_runtime *ctx,
				uint16_t vlan_tag)
{
#if defined(CONFIG_NET_VLAN)
	struct net_if *iface;

	iface = net_eth_get_vlan_iface(ctx->iface, vlan_tag);
	if (!iface) {
		return ctx->iface;
	}

	return iface;
#else
	ARG_UNUSED(vlan_tag);

	return ctx->iface;
#endif
}

static int eth_xmc4xxx_send(struct device *dev, struct net_pkt *pkt)
{
	struct eth_xmc4xxx_runtime *dev_data = DEV_DATA(dev);
	uint8_t *dma_buffer;
	int res;
	size_t total_len;

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(dev_data != NULL);

	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);

	total_len = net_pkt_get_len(pkt);
	if (total_len > ETH_TX_BUF_SIZE) {
		LOG_ERR("PKT to big");
		res = -EIO;
		goto error;
	}
    while (XMC_ETH_MAC_IsTxDescriptorOwnedByDma(&eth_mac)) {
        XMC_ETH_MAC_ResumeTx(&eth_mac);
        k_yield();
    }

	dma_buffer = XMC_ETH_MAC_GetTxBuffer(&eth_mac);
	if (net_pkt_read(pkt, dma_buffer, total_len)) {
		res = -EIO;
		goto error;
	}
    XMC_ETH_MAC_SetTxBufferSize(&eth_mac, total_len);
    XMC_ETH_MAC_ReturnTxDescriptor(&eth_mac);
    XMC_ETH_MAC_ResumeTx(&eth_mac);
 
	res = 0;
error:

	k_mutex_unlock(&dev_data->tx_mutex);

	return res;
}

static void eth_xmc4xxx_rx_error(struct net_if *iface)
{
	//struct device *dev = net_if_get_device(iface);

	eth_stats_update_errors_rx(iface);

}

static struct net_pkt *eth_xmc4xxx_rx_pkt(struct device *dev,
					    struct net_if *iface, 
						uint16_t *vlan_tag)
{
	struct eth_xmc4xxx_runtime *dev_data;
	struct net_pkt *pkt = NULL;
	uint8_t buffer[ETH_RX_BUF_SIZE];
	size_t total_len;
	__ASSERT_NO_MSG(dev != NULL);
	dev_data = DEV_DATA(dev);
	__ASSERT_NO_MSG(dev_data != NULL);

/*
		len = XMC_ETH_MAC_GetRxFrameSize(&eth_mac);
		
*/
    if (XMC_ETH_MAC_IsRxDescriptorOwnedByDma(&eth_mac) == false) {
        /* while (1) { */
        //uint8_t *buf_eth;
        total_len = XMC_ETH_MAC_GetRxFrameSize(&eth_mac);
        if ((total_len < 0) || (total_len>ETH_RX_BUF_SIZE))
		{
			total_len = XMC_ETH_MAC_GetRxFrameSize(&eth_mac);
			XMC_ETH_MAC_ReturnRxDescriptor(&eth_mac);  
    		XMC_ETH_MAC_ResumeRx(&eth_mac);
			goto done;
		}
		XMC_ETH_MAC_ReadFrame(&eth_mac, &buffer[0], total_len);
    	XMC_ETH_MAC_ResumeRx(&eth_mac);
        /*buf_eth = XMC_ETH_MAC_GetRxBuffer(&eth_mac);
        if (!buf_eth)
            goto done;
		*/
	
		pkt = net_pkt_rx_alloc_with_buffer(get_iface(dev_data, *vlan_tag),
						total_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("Failed to obtain RX buffer");
			goto done;
		}

		if (net_pkt_write(pkt, (&buffer[0]), total_len)) {
			LOG_ERR("Failed to append RX buffer to context buffer");
			net_pkt_unref(pkt);
			pkt = NULL;
			goto done;
		}
    }
done:
    return pkt;

}

static void eth_xmc4xxx_rx(struct device *dev)
{
	uint16_t vlan_tag = NET_VLAN_TAG_UNSPEC;
	struct eth_xmc4xxx_runtime *dev_data = DEV_DATA(dev);
	struct net_if *iface = dev_data->iface;
	struct net_pkt *pkt = NULL;

	pkt = eth_xmc4xxx_rx_pkt(dev, iface, &vlan_tag);
	if (!pkt) {
		LOG_ERR("Failed to read data");
		goto err_mem;
	}
#if defined(CONFIG_NET_VLAN)
	struct net_eth_hdr *hdr = NET_ETH_HDR(pkt);

	if (ntohs(hdr->type) == NET_ETH_PTYPE_VLAN) {
		struct net_eth_vlan_hdr *hdr_vlan =
			(struct net_eth_vlan_hdr *)NET_ETH_HDR(pkt);

		net_pkt_set_vlan_tci(pkt, ntohs(hdr_vlan->vlan.tci));
		vlan_tag = net_pkt_vlan_tag(pkt);

#if CONFIG_NET_TC_RX_COUNT > 1
		enum net_priority prio;

		prio = net_vlan2priority(net_pkt_vlan_priority(pkt));
		net_pkt_set_priority(pkt, prio);
#endif
	} else {
		net_pkt_set_iface(pkt, dev_data->iface);
	}
#endif /* CONFIG_NET_VLAN */

	if (net_recv_data(iface, pkt) < 0) {
		LOG_ERR("Failed to place frame in RX Queue");
		goto pkt_unref;
	}

	return;

pkt_unref:
	net_pkt_unref(pkt);

err_mem:
	eth_xmc4xxx_rx_error(iface);
}

static void eth_xmc4xxx_isr(void *arg)
{
	/* Read the interrupt status */
	struct device *dev = (struct device *)arg;
	//struct eth_xmc4xxx_runtime *dev_data = DEV_DATA(dev);
	uint32_t lock;

	lock = irq_lock();
  	uint32_t status;

	status = XMC_ETH_MAC_GetEventStatus(&eth_mac);

	if (status & XMC_ETH_MAC_EVENT_RECEIVE)
	{
		XMC_ETH_MAC_DisableEvent(&eth_mac, XMC_ETH_MAC_EVENT_RECEIVE);
		eth_xmc4xxx_rx(dev);
		XMC_ETH_MAC_EnableEvent(&eth_mac, XMC_ETH_MAC_EVENT_RECEIVE);
	}

	XMC_ETH_MAC_ClearEventStatus(&eth_mac, status);

	/* Acknowledge the interrupt. */
	irq_unlock(lock);
}

static void eth_xmc4xxx_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	//const struct eth_xmc4xxx_config *dev_conf = DEV_CFG(dev);
	struct eth_xmc4xxx_runtime *dev_data = DEV_DATA(dev);
	const struct eth_xmc4xxx_config *dev_conf = DEV_CFG(dev);
	dev_data->iface = iface;

	/* Assign link local address. */
	net_if_set_link_addr(iface,
			     dev_data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* Initialize semaphore. */
	k_sem_init(&dev_data->tx_sem, 0, 1);
	k_mutex_init(&dev_data->tx_mutex);

	/* Initialize Interrupts. */
	dev_conf->config_func(dev);

    XMC_ETH_LINK_SPEED_t speed;
    XMC_ETH_LINK_DUPLEX_t duplex;
    bool phy_autoneg_state = false;
    uint32_t retries = 0;
    if (XMC_ETH_PHY_GetLinkStatus(&eth_mac, ETH_LWIP_0_PHY_ADDR) != XMC_ETH_LINK_STATUS_DOWN) {
        XMC_ETH_PHY_Init(&eth_mac, ETH_LWIP_0_PHY_ADDR, &eth_phy_config);
        do {
            phy_autoneg_state = XMC_ETH_PHY_IsAutonegotiationCompleted(&eth_mac, ETH_LWIP_0_PHY_ADDR);
            retries++;
        } while ((phy_autoneg_state == false) && (retries < ETH_LWIP_PHY_MAX_RETRIES));
    }
    
    if(phy_autoneg_state == false)
        return;
    speed = XMC_ETH_PHY_GetLinkSpeed(&eth_mac, ETH_LWIP_0_PHY_ADDR);
    duplex = XMC_ETH_PHY_GetLinkDuplex(&eth_mac, ETH_LWIP_0_PHY_ADDR);
	
    XMC_ETH_MAC_SetLink(&eth_mac, speed, duplex);
    /* Enable ethernet interrupts */
    XMC_ETH_MAC_EnableEvent(&eth_mac, (uint32_t)XMC_ETH_MAC_EVENT_RECEIVE);
    XMC_ETH_MAC_EnableTx(&eth_mac);
    XMC_ETH_MAC_EnableRx(&eth_mac);
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_xmc4xxx_stats(struct device *dev)
{
	return &(DEV_DATA(dev)->stats);
}
#endif

static int eth_xmc4xxx_dev_init(struct device *dev)
{

    XMC_ETH_MAC_PORT_CTRL_t port_control;
    XMC_GPIO_CONFIG_t gpio_config;
    gpio_config.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_CRS_DV, &gpio_config);
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_RXER, &gpio_config);
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_RXD0, &gpio_config);
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_RXD1, &gpio_config);
    gpio_config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE;
    gpio_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT1;
    XMC_GPIO_Init(ETH_LWIP_0_TXEN, &gpio_config);
    gpio_config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE;
    gpio_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT1;
    XMC_GPIO_Init(ETH_LWIP_0_TXD0, &gpio_config);
    gpio_config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE;
    gpio_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT1;
    XMC_GPIO_Init(ETH_LWIP_0_TXD1, &gpio_config);
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_RMII_CLK, &gpio_config);
    gpio_config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE;
    gpio_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT1;
    XMC_GPIO_Init(ETH_LWIP_0_MDC, &gpio_config);
    gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(ETH_LWIP_0_MDIO, &gpio_config);
    XMC_GPIO_SetHardwareControl(ETH_LWIP_0_MDIO, XMC_GPIO_HWCTRL_PERIPHERAL1);
    port_control.mode = XMC_ETH_MAC_PORT_CTRL_MODE_RMII;
    port_control.rxd0 = (XMC_ETH_MAC_PORT_CTRL_RXD0_t)0U;
    port_control.rxd1 = (XMC_ETH_MAC_PORT_CTRL_RXD1_t)0U;
    port_control.clk_rmii = (XMC_ETH_MAC_PORT_CTRL_CLK_RMII_t)2U;
    port_control.crs_dv = (XMC_ETH_MAC_PORT_CTRL_CRS_DV_t)2U;
    port_control.rxer = (XMC_ETH_MAC_PORT_CTRL_RXER_t)0U;
    port_control.mdio = (XMC_ETH_MAC_PORT_CTRL_MDIO_t)1U;
    XMC_ETH_MAC_SetPortControl(&eth_mac, port_control);
    (void)XMC_ETH_MAC_Init(&eth_mac);
	/* Assign MAC address to Hardware */
	eth_xmc4xxx_assign_mac(dev);
    XMC_ETH_MAC_DisableJumboFrame(&eth_mac);
    XMC_ETH_MAC_EnableReceptionBroadcastFrames(&eth_mac);

	return 0;
}

DEVICE_DECLARE(eth_xmc4xxx);

static void eth_xmc4xxx_irq_config(struct device *dev)
{
	/* Enable Interrupt. */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    eth_xmc4xxx_isr, DEVICE_GET(eth_xmc4xxx), 0);
	NVIC_SetPriority(ETH0_0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 63U, 0U));
	irq_enable(DT_INST_IRQN(0));
}

struct eth_xmc4xxx_config eth_cfg = {
	.mac_base = DT_INST_REG_ADDR(0),
	.config_func = eth_xmc4xxx_irq_config,
};

struct eth_xmc4xxx_runtime eth_data = {
	.mac_addr = DT_INST_PROP(0, local_mac_address),
	.tx_err = false,
	.tx_word = 0,
	.tx_pos = 0,
};

static const struct ethernet_api eth_xmc4xxx_apis = {
	.iface_api.init	= eth_xmc4xxx_init,
	.send =  eth_xmc4xxx_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_xmc4xxx_stats,
#endif
};

NET_DEVICE_INIT(eth_xmc4xxx, DT_INST_LABEL(0),
		eth_xmc4xxx_dev_init, device_pm_control_nop,
		&eth_data, &eth_cfg, CONFIG_ETH_INIT_PRIORITY,
		&eth_xmc4xxx_apis, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
