/* XMC4XXX Ethernet Controller
 *
 * Copyright (c) 2020 SLB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ETH_XMC4XXX_PRIV_H_
#define ETH_XMC4XXX_PRIV_H_

#define ETH_RX_BUF_SIZE	XMC_ETH_MAC_BUF_SIZE /* buffer size for receive */
#define ETH_TX_BUF_SIZE	XMC_ETH_MAC_BUF_SIZE /* buffer size for transmit */

#define DEV_DATA(dev) \
	((struct eth_xmc4xxx_runtime *)(dev)->driver_data)
#define DEV_CFG(dev) \
	((const struct eth_xmc4xxx_config *const)(dev)->config_info)


struct eth_xmc4xxx_runtime {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_sem tx_sem;
	struct k_mutex tx_mutex;
	bool tx_err;
	uint32_t tx_word;
	int tx_pos;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

typedef void (*eth_xmc4xxx_config_irq_t)(struct device *dev);

struct eth_xmc4xxx_config {
	uint32_t mac_base;
	uint32_t sys_ctrl_base;
	uint32_t irq_num;
	eth_xmc4xxx_config_irq_t config_func;
};

#endif /* ETH_XMC4XXX_PRIV_H_ */
