#ifndef __AG71XX_SLAVES_H
#define __AG71XX_SLAVES_H

//для tx это номер порта
#define AR7240_TX_HEADER_SOURCE_PORT_M BITM(3)
//для rx это битовая маска!
#define AR7240_RX_HEADER_SOURCE_PORT_M BITM(6)


static inline struct net_device *get_slave_dev_by_port_num(struct ag71xx *master_ag, u32 port_num)
{
	if(likely(port_num < master_ag->slave_devs_count))
		return master_ag->slave_devs[port_num];
	else
		return NULL;
}

static inline void set_slave_dev_by_port_num(struct ag71xx *master_ag, u32 port_num, struct net_device *dev)
{
	if(likely(port_num < master_ag->slave_devs_count))
		master_ag->slave_devs[port_num] = dev;
	else
		printk(KERN_ERR "%s: port num := %u is wrong !!!\n", __func__, port_num);
}

static inline struct ag71xx_slave *get_slave_ags_by_port_num(struct ag71xx *master_ag, u32 port_num){
	if(likely(port_num < master_ag->slave_devs_count && master_ag->slave_devs[port_num]))
		return netdev_priv(master_ag->slave_devs[port_num]);
	return NULL;
}

void create_slave_devices(struct ag71xx *ag);
void destroy_slave_devices(struct ag71xx *ag);

#endif /* __AG71XX_SLAVES_H */
