#include <linux/switch.h>
#include "ag71xx.h"
#include "ag71xx_cross_switch.h"
#include "ag71xx_slaves.h"

/* Здесь собран кросс свитчевый код */


/* Вспомогательная ф-я вызываемая из ar8327 для создания slave устройств.
	 Это как бы прослойка-адаптер между set_iface_mode и вызовом create_slave_devices. */
int ag71xx_ext_sw_create_slave_devices(struct net_device *net_dev, struct switch_dev *sw_dev)
{
	struct ag71xx *ag = netdev_priv(net_dev);
	if(unlikely(!ag))
		return -1;
	ag->phy_swdev = sw_dev;
	ag->sw_ver = EXT_SW_VERSION_AR8327;
	create_slave_devices(ag);
	return 0;
}

/* ф-я установки состояния порта: on/off. */
void ag71xx_cross_sw_set_port_state(struct ag71xx *ag, unsigned port, int state){
	//printk(KERN_DEBUG "%s: port = %d, state = %d, ag->sw_ver = 0x%x\n", __func__, port, state, ag->sw_ver);
	if(ag->sw_ver == EXT_SW_VERSION_AR8327){
		ar8327_set_port_state(ag->phy_swdev, port, state);
	}else{
		ag71xx_ar7240_set_port_state(ag, port, state);
	}
}

/* универсальная ф-я определения кол-ва портов у свитча */
int ar71xx_get_sw_num_ports(struct ag71xx *ag)
{
	if(!ag)
		return 0;
	if(ag->phy_swdev)
		return ag->phy_swdev->ports;
	else
		return ag71xx_ar7240_get_num_ports(ag);
}

/* кросс свитчевые ф-и для чтения и записи регистров mdio */
u16 ag71xx_cross_sw_mdio_read(struct ag71xx *master_ag, unsigned phy_addr, unsigned reg_addr){
	struct mii_bus *mii;
	if(master_ag->sw_ver == EXT_SW_VERSION_AR8327){
		mii = ar8327_get_mii_bus_from_swdev(master_ag->phy_swdev);
		return mdiobus_read(mii, phy_addr, reg_addr);
	}else{
		mii = master_ag->mii_bus;
		return ar7240sw_phy_read(mii, phy_addr, reg_addr);
	}
}
int ag71xx_cross_sw_mdio_write(struct ag71xx *master_ag, unsigned phy_addr, unsigned reg_addr, u16 reg_val){
	struct mii_bus *mii;
	if(master_ag->sw_ver == EXT_SW_VERSION_AR8327){
		mii = ar8327_get_mii_bus_from_swdev(master_ag->phy_swdev);
		return mdiobus_write(mii, phy_addr, reg_addr, reg_val);
	}else{
		mii = master_ag->mii_bus;
		return ar7240sw_phy_write(mii, phy_addr, reg_addr, reg_val);
	}
}

/* применяется для ручной установки скорости на портах */
int ag71xx_cross_sw_adjust_port_link(struct ag71xx *master_ag, u32 port, struct switch_port_link *port_link, u8 advertise){
	int phy_addr = port - 1;
	u16 bmcr = 0;
	u32 aneg_adv = 0;
	u32 aneg1000_adv = 0;
	bool aneg_need_reset = 0;
	int ret = 0;
  struct ag71xx_slave *ags = get_slave_ags_by_port_num(master_ag, port);
	struct switch_port_link fake_port_link;

	/* в ags мы храним установки скорости */
	if(!ags)
		return -EINVAL;

	/* если это режим установки скорости */
	if(port_link){
		ags->adj_speed = port_link->speed;
		ags->adj_duplex = port_link->duplex;
		ags->adj_aneg = port_link->aneg;
		ags->need_adjust = 1;
	}else{
		/* режим восстановления(применения) ранее сохраненных
			 значений. используется в port_setup. */
		fake_port_link.speed = ags->adj_speed;
		fake_port_link.duplex = ags->adj_duplex;
		fake_port_link.aneg = ags->adj_aneg;
		port_link = &fake_port_link;
	}

	//пропускаем самый первый проход
	if(!ags->need_adjust)
		return 0;

	/*printk(KERN_DEBUG "%s/%s: port = %u speed = %u, duplex = %u, aneg = %u, advertise=0x%x\n",
				 __func__, ags->dev->name, port, port_link->speed,
				 port_link->duplex, port_link->aneg, advertise); */
	/* printk(KERN_DEBUG "%s/%s: do: MII_BMCR = 0x%x\n",
		__func__, ags->dev->name, ag71xx_cross_sw_mdio_read(master_ag, phy_addr, MII_BMCR));
	printk(KERN_DEBUG "%s/%s: do: MII_ADVERTISE = 0x%x\n",
		__func__, ags->dev->name, ag71xx_cross_sw_mdio_read(master_ag, phy_addr, MII_ADVERTISE)); */

	/* нам переданы биты для создания набора advertise скоростей */
	if(advertise){
		int v = advertise & 0xFF;
		aneg_need_reset = 1;
		/* это регистр для 10-100 мегабит */
		aneg_adv = ag71xx_cross_sw_mdio_read(master_ag, phy_addr, MII_ADVERTISE);
		/* а это отдельно регистр для 1000 мегабит */
		aneg1000_adv = ag71xx_cross_sw_mdio_read(master_ag, phy_addr, MII_CTRL1000);
		//10half
		if(v & 0x1)
			aneg_adv |= ADVERTISE_10HALF;
		else
			aneg_adv &= ~ADVERTISE_10HALF;
		//10full
		if(v & 0x2)
			aneg_adv |= ADVERTISE_10FULL;
		else
			aneg_adv &= ~ADVERTISE_10FULL;
		//100half
		if(v & 0x4)
			aneg_adv |= ADVERTISE_100HALF;
		else
			aneg_adv &= ~ADVERTISE_100HALF;
		//100full
		if(v & 0x8)
			aneg_adv |= ADVERTISE_100FULL;
		else
			aneg_adv &= ~ADVERTISE_100FULL;
		//1000half
		if(v & 0x10)
			aneg1000_adv |= ADVERTISE_1000HALF;
		else
			aneg1000_adv &= ~ADVERTISE_1000HALF;
		//1000full
		if(v & 0x20)
			aneg1000_adv |= ADVERTISE_1000FULL;
		else
			aneg1000_adv &= ~ADVERTISE_1000FULL;

		//printk(KERN_DEBUG "aneg_adv = 0x%x\n", aneg_adv);
	}

	ag71xx_cross_sw_mdio_write(master_ag, phy_addr, MII_BMCR, 0x0000);
	if(port_link->aneg){
		bmcr = BMCR_ANENABLE | BMCR_ANRESTART;
	}else{
		if(port_link->duplex)
			bmcr |= BMCR_FULLDPLX;
		switch(ags->speed){
			case SWITCH_PORT_SPEED_10:
				break;
			case SWITCH_PORT_SPEED_100:
				bmcr |= BMCR_SPEED100;
				break;
			case SWITCH_PORT_SPEED_1000:
				bmcr |= BMCR_SPEED1000;
			default:
				ret = -ENOTSUPP;
				goto end;
		}
	}
	if(aneg_need_reset){
		ag71xx_cross_sw_mdio_write(master_ag, phy_addr, MII_ADVERTISE, aneg_adv);
		ag71xx_cross_sw_mdio_write(master_ag, phy_addr, MII_CTRL1000, aneg1000_adv);
	}
	ag71xx_cross_sw_mdio_write(master_ag, phy_addr, MII_BMCR, bmcr | BMCR_RESET);
end:
	return ret;
}

/* общий для всех свитчей код ф-и set_port_link - ручной установки скорости для порта */
int cross_sw_set_port_link(struct ag71xx *master_ag, struct switch_dev *dev, int port,
struct switch_port_link *port_link, u8 advertise)
{
  struct ag71xx_slave *ags = get_slave_ags_by_port_num(master_ag, port);
	int ret = 0;

	if(!ags)
		return -EINVAL;

	if (port == dev->cpu_port)
		return -EINVAL;

	if (port >= dev->ports)
		return -EINVAL;

	cancel_delayed_work_sync(&ags->link_work);

	ags->speed = port_link->speed;
	ags->duplex = port_link->duplex;
	ags->aneg = port_link->aneg;

	ret = ag71xx_cross_sw_adjust_port_link(master_ag, port, port_link, advertise);

	schedule_delayed_work(&ags->link_work, HZ / 10);
	return ret;
}
