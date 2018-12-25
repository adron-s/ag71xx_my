#include <linux/switch.h>
#include "ag71xx.h"
#include "ag71xx_cross_switch.h"
#include "ag71xx_slaves.h"

#define DEV_ADDR_ADD(dev_addr, _v){						\
	int v = _v;																	\
	unsigned char *p = &dev_addr[ETH_ALEN - 1];	\
	while(p >= dev_addr){												\
		unsigned char prev = (*p);								\
		(*p)+= v;																	\
		v = 1;																		\
		if(prev > *p)															\
			p--;																		\
		else																			\
			break;																	\
	}																						\
}

static netdev_tx_t slave_dev_hard_start_xmit(struct sk_buff *skb,
			 struct net_device *dev)
{
	int ret;
	int len;
  struct ag71xx_slave *ags = netdev_priv(dev);
	struct net_device *master_dev = ags->master_ag->dev;
	if(unlikely(skb_cloned(skb) || skb_shared(skb))){
		struct sk_buff *old_skb = skb;
		skb = skb_copy(old_skb, GFP_ATOMIC);
		kfree_skb(old_skb);
		if(!skb){
			dev->stats.tx_dropped++;
			return NET_XMIT_DROP;
		}
	}
	ags->ag71xx_add_atheros_header(skb, ags->ath_hdr_port_byte);
	skb->dev = master_dev;
	len = skb->len;
	//вставляем нашу skb в очередь на передачу устройства master_dev
	ret = dev_queue_xmit(skb);
	if(likely(ret == NET_XMIT_SUCCESS)){
		dev->stats.tx_bytes += len;
		dev->stats.tx_packets++;
	}else
		dev->stats.tx_dropped++;
	return ret;
}

static int slave_dev_open(struct net_device *dev)
{
	int ret = 0;
  struct ag71xx_slave *ags = netdev_priv(dev);
	struct net_device *master_dev = ags->master_ag->dev;
	schedule_delayed_work(&ags->link_work, HZ / 10);
	/* важно сначала поднять master интерфейс
		 так как он проинитит свитч ! */
	ret = dev_open(master_dev);
	if(!ret){
		ag71xx_cross_sw_set_port_state(ags->master_ag, ags->port_num, 1);
		netif_start_queue(dev);
	}
	return ret;
}

static int slave_dev_stop(struct net_device *dev)
{
  struct ag71xx_slave *ags = netdev_priv(dev);
	cancel_delayed_work_sync(&ags->link_work);
	netif_stop_queue(dev);
	ag71xx_cross_sw_set_port_state(ags->master_ag, ags->port_num, 0);
	return 0;
}

static void slave_link_function(struct work_struct *work)
{
	struct ag71xx_slave *ags = container_of(work, struct ag71xx_slave, link_work.work);
	struct switch_port_link port_link;
	struct switch_dev *swdev;
	if(ags->master_ag && ags->master_ag->phy_swdev)
		swdev = ags->master_ag->phy_swdev;
	else
		swdev = ag71xx_ar7240_get_swdev(ags->master_ag);
	if(!swdev || !swdev->ops || !swdev->ops->get_port_link){
		printk(KERN_ERR "BUG! %s: 0x%p, 0x%p, 0x%p\n",
			__func__, swdev, swdev->ops, swdev->ops->get_port_link);
		return;
	}
	memset(&port_link, '\0', sizeof(port_link));
	if(!swdev->ops->get_port_link(swdev, ags->port_num, &port_link)){
		ags->speed = port_link.speed;
		ags->duplex = port_link.duplex;
		ags->aneg = port_link.aneg;
		if(port_link.link != ags->link){
			ags->link = port_link.link;
			if(!ags->link){ //if link is DOWN
				netif_carrier_off(ags->dev);
				if(netif_msg_link(ags->master_ag))
					pr_info("%s: link down\n", ags->dev->name);
			}else{ //if link is UP
				netif_carrier_on(ags->dev);
				if(netif_msg_link(ags->master_ag))
					pr_info("%s: link up(%d, %d, %d)\n", ags->dev->name,
						port_link.speed, port_link.duplex, port_link.aneg);
			}
		}
	}
	schedule_delayed_work(&ags->link_work, HZ / 2);
}

static const struct net_device_ops slave_dev_netdev_ops = {
	.ndo_open		= slave_dev_open,
	.ndo_stop		= slave_dev_stop,
	.ndo_start_xmit		= slave_dev_hard_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int create_slave_device(struct ag71xx *master_ag, u32 port_num, int sw_ver){
	struct platform_device *pdev = master_ag->pdev;
	struct net_device *dev = NULL;
	char dev_name[sizeof(dev->name)];
	int err = 0;
	struct ag71xx_slave *ags;
	dev_hold(master_ag->dev);
	if(port_num & ~AR7240_TX_HEADER_SOURCE_PORT_M){
		printk(KERN_ERR "BUG: %s: Incorrect port_num %d\n", __func__, port_num);
		err = -EINVAL;
		goto err_out;
	}

	if(get_slave_dev_by_port_num(master_ag, port_num)){
		printk(KERN_ERR "BUG: %s: net_device(%s) for sw_port %d already exists",
			__func__, get_slave_dev_by_port_num(master_ag, port_num)->name, port_num);
		err = -EINVAL;
		goto err_out;
	}

	snprintf(dev_name, sizeof(dev_name), "%sp%d", master_ag->dev->name, port_num);
	dev = alloc_netdev_mqs(sizeof(*ags), dev_name, NET_NAME_UNKNOWN,
		ether_setup, 1, 1);

	if (!dev) {
		dev_err(&pdev->dev, "alloc_etherdev for sw_port %d failed\n", port_num);
		err = -ENOMEM;
		goto err_out;
	}
	ags = netdev_priv(dev);
	memset(ags, 0x0, sizeof(*ags));
	spin_lock_init(&ags->lock);
	ags->is_master = 0;
	ags->port_num = port_num;
	ags->master_ag = master_ag;
	/* ags->port_num и ags->master_ag должны быть уже заданы! */
	if(ag71xx_ar7240_cook_ags_depending_on_the_sw_ver(ags, sw_ver)){
		err = -EINVAL;
		dev_err(&pdev->dev, "unable to cook ags for sw_port %d\n", port_num);
		goto err;
	}
	/* в начале ставим статус линка NO LINK. если это не так то
		 slave_link_function при своем следующем вызове его поправит.
		 и netif_carrier_off(dev) правильно умеет отрабатывать установку
		 несущей до вызова register_netdev! он проверяет что статус ==
		 NETREG_UNINITIALIZED и не генерирует событие об изменении статуса
		 а просто тихонько правит нужный бит в dev->... */
	ags->link = 0;
	ags->speed = 0;
	ags->duplex = 0;
	netif_carrier_off(dev);
	ag71xx_cross_sw_set_port_state(master_ag, ags->port_num, 0);
	//dev->base_addr = (unsigned long)master_ag->mac_base + port_num;
	dev->netdev_ops = &slave_dev_netdev_ops;
	memcpy(dev->dev_addr, master_ag->dev->dev_addr, ETH_ALEN);
	DEV_ADDR_ADD(dev->dev_addr, port_num - 1);
	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "unable to register net device for sw_port %d\n", port_num);
		goto err;
	}
	ags->dev = dev;
	//индексирование в ag71xx->slave_devs по номеру порта! не путать с битовой маской!
  set_slave_dev_by_port_num(master_ag, port_num, dev);
	INIT_DELAYED_WORK(&ags->link_work, slave_link_function);
  return 0;
err:
err_out:
	dev_put(master_ag->dev);
	if(dev)
		free_netdev(dev);
	return err;
}

static void destroy_slave_device(struct ag71xx *master_ag, u32 port_num){
	struct net_device *dev = get_slave_dev_by_port_num(master_ag, port_num);
	if(dev){
		struct ag71xx_slave *ags = netdev_priv(dev);
		cancel_delayed_work_sync(&ags->link_work);
		netif_tx_disable(dev);
		unregister_netdev(dev);
		dev_put(master_ag->dev);
		free_netdev(dev);
		set_slave_dev_by_port_num(master_ag, port_num, NULL);
	}
}

void create_slave_devices(struct ag71xx *ag){
	int a;
	if(ag->has_slaves){
		printk(KERN_DEBUG "%s: Already has_slaves ! Do nothing !\n", __func__);
		return;
	}
	printk(KERN_DEBUG "%s: sw_ver = 0x%x\n", __func__, ag->sw_ver);
	ag->has_slaves = 1;
	ag->slave_devs_count = ar71xx_get_sw_num_ports(ag);
	ag->slave_devs = kzalloc(sizeof(ag->slave_devs[0]) * ag->slave_devs_count, GFP_KERNEL);
	if(!ag->slave_devs){
		ag->slave_devs_count = 0;
		printk(KERN_ERR "%s: Can't alloc memory for ag->slave_devs !!!\n", __func__);
		return;
	}
	for(a = 1; a < ar71xx_get_sw_num_ports(ag); a++)
 		create_slave_device(ag, a, ag->sw_ver);
	for(a = 0; a < ag->slave_devs_count; a++){
		struct net_device *sdd = get_slave_dev_by_port_num(ag, a);
		printk(KERN_DEBUG "sw port %d -> %s -> 0x%x\n", a,
			sdd ? sdd->name : "EMPTY",
			sdd ? ((struct ag71xx_slave *)netdev_priv(
			sdd))->ath_hdr_port_byte : -1);
	}
}

void destroy_slave_devices(struct ag71xx *ag){
	int a;
	//printk(KERN_DEBUG "%s: DESTROY !!!\n", __func__);
	for(a = 1; a < ag->slave_devs_count; a++)
 		destroy_slave_device(ag, a);
 	if(ag->slave_devs){
 		ag->slave_devs_count = 0;
 		kfree(ag->slave_devs);
 		ag->slave_devs = NULL;
 	}
}
