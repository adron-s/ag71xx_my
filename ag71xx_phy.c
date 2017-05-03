/*
 *  Atheros AR71xx built-in ethernet mac driver
 *
 *  Copyright (C) 2008-2010 Gabor Juhos <juhosg@openwrt.org>
 *  Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 *
 *  Based on Atheros' AG7100 driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include "ag71xx.h"

static void ag71xx_phy_link_adjust(struct net_device *dev)
{
	struct ag71xx *ag = netdev_priv(dev);
	struct phy_device *phydev = ag->phy_dev;
	unsigned long flags;
	int status_change = 0;

	spin_lock_irqsave(&ag->lock, flags);

	if (phydev->link) {
		if (ag->duplex != phydev->duplex
		    || ag->speed != phydev->speed) {
			status_change = 1;
		}
	}

	if (phydev->link != ag->link)
		status_change = 1;

	ag->link = phydev->link;
	ag->duplex = phydev->duplex;
	ag->speed = phydev->speed;

	if (status_change)
		ag71xx_link_adjust(ag);

	spin_unlock_irqrestore(&ag->lock, flags);
}

static void ag71xx_phy_link_adjust_for_slave(struct net_device *dev)
{
	struct ag71xx_slave *ags = netdev_priv(dev);
	struct phy_device *phydev = ags->phy_dev;
	unsigned long flags;
	int status_change = 0;

	spin_lock_irqsave(&ags->lock, flags);

	if (phydev->link) {
		if (ags->duplex != phydev->duplex
		    || ags->speed != phydev->speed) {
			status_change = 1;
		}
	}

	if (phydev->link != ags->link)
		status_change = 1;

	ags->link = phydev->link;
	ags->duplex = phydev->duplex;
	ags->speed = phydev->speed;

	//if (status_change)
		//ag71xx_link_adjust(ags);

	spin_unlock_irqrestore(&ags->lock, flags);
}

void ag71xx_phy_set_phy_state(struct ag71xx *ag, int state)
{
	u32 bmcr = 0;
	if(state){
		bmcr &= ~BMCR_PDOWN; //включаем питание на трансивер
		//на всякий случай еще рестартанем трансивер и автонеготинацию
		bmcr |= BMCR_RESET | BMCR_ANENABLE;
	}else{
		//отрубаем трансивер
		bmcr |= BMCR_PDOWN;
	}
	printk(KERN_DEBUG "%s for dev := '%s', phy_dev = 0x%p\n", __func__, ag->dev->name, ag->phy_dev);
	if(ag->phy_dev){
		phy_write(ag->phy_dev, MII_BMCR, bmcr);
	}
}

void ag71xx_phy_start(struct ag71xx *ag)
{
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);

	printk(KERN_DEBUG "%s for dev := '%s'\n", __func__, ag->dev->name);
	if (ag->phy_dev) {
		ag71xx_phy_set_phy_state(ag, 1);
		phy_start(ag->phy_dev);
	} else if (pdata->mii_bus_dev && pdata->switch_data) {
		ag71xx_ar7240_start(ag);
	} else {
		ag->link = 1;
		ag71xx_link_adjust(ag);
	}
}

void ag71xx_phy_stop(struct ag71xx *ag)
{
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);
	unsigned long flags;

	printk(KERN_DEBUG "%s for dev := '%s'\n", __func__, ag->dev->name);

	if (ag->phy_dev){
		phy_stop(ag->phy_dev);
		ag71xx_phy_set_phy_state(ag, 0);
	}
	else if (pdata->mii_bus_dev && pdata->switch_data)
		ag71xx_ar7240_stop(ag);

	spin_lock_irqsave(&ag->lock, flags);
	if (ag->link) {
		ag->link = 0;
		ag71xx_link_adjust(ag);
	}
	spin_unlock_irqrestore(&ag->lock, flags);
}

static int ag71xx_phy_connect_fixed(struct ag71xx *ag)
{
	struct device *dev = &ag->pdev->dev;
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);
	int ret = 0;

	/* use fixed settings */
	switch (pdata->speed) {
	case SPEED_10:
	case SPEED_100:
	case SPEED_1000:
		break;
	default:
		dev_err(dev, "invalid speed specified\n");
		ret = -EINVAL;
		break;
	}

	dev_dbg(dev, "using fixed link parameters\n");

	ag->duplex = pdata->duplex;
	ag->speed = pdata->speed;

	return ret;
}

static int ag71xx_phy_connect_multi(struct ag71xx *ag)
{
	struct device *dev = &ag->pdev->dev;
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);
	struct phy_device *phydev = NULL;
	int phy_addr;
	int ret = 0;

	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (!(pdata->phy_mask & (1 << phy_addr)))
			continue;

		if (ag->mii_bus->phy_map[phy_addr] == NULL)
			continue;

		DBG("%s: PHY found at %s, uid=%08x\n",
			dev_name(dev),
			dev_name(&ag->mii_bus->phy_map[phy_addr]->dev),
			ag->mii_bus->phy_map[phy_addr]->phy_id);

		if (phydev == NULL)
			phydev = ag->mii_bus->phy_map[phy_addr];
	}

	if (!phydev) {
		dev_err(dev, "no PHY found with phy_mask=%08x\n",
			   pdata->phy_mask);
		return -ENODEV;
	}

	ag->phy_dev = phy_connect(ag->dev, dev_name(&phydev->dev),
				  &ag71xx_phy_link_adjust,
				  pdata->phy_if_mode);

	if (IS_ERR(ag->phy_dev)) {
		dev_err(dev, "could not connect to PHY at %s\n",
			   dev_name(&phydev->dev));
		return PTR_ERR(ag->phy_dev);
	}

	/* mask with MAC supported features */
	if (pdata->has_gbit)
		phydev->supported &= PHY_GBIT_FEATURES;
	else
		phydev->supported &= PHY_BASIC_FEATURES;

	phydev->advertising = phydev->supported;

	dev_info(dev, "connected to PHY at %s [uid=%08x, driver=%s]\n",
		    dev_name(&phydev->dev), phydev->phy_id, phydev->drv->name);

	ag->link = 0;
	ag->speed = 0;
	ag->duplex = -1;

  ag71xx_phy_set_phy_state(ag, 0); //initial state для WAN порта!

	return ret;
}

static int dev_is_class(struct device *dev, void *class)
{
	if (dev->class != NULL && !strcmp(dev->class->name, class))
		return 1;

	return 0;
}

static struct device *dev_find_class(struct device *parent, char *class)
{
	if (dev_is_class(parent, class)) {
		get_device(parent);
		return parent;
	}

	return device_find_child(parent, class, dev_is_class);
}

static struct mii_bus *dev_to_mii_bus(struct device *dev)
{
	struct device *d;

	d = dev_find_class(dev, "mdio_bus");
	if (d != NULL) {
		struct mii_bus *bus;

		bus = to_mii_bus(d);
		put_device(d);

		return bus;
	}

	return NULL;
}

void ag71xx_phy_connect_for_slaves(struct ag71xx_slave *ags){
	struct phy_device *phydev = ag71xx_ar7240_get_phydev_for_slave(ags);
	if(phydev){
		printk(KERN_DEBUG "%s: PHY found at %s, uid=%08x, irq=0x%x\n",
		ags->dev->name,
		dev_name(&phydev->dev),
		phydev->phy_id, phydev->irq);
		ags->phy_dev = phy_connect(ags->dev, dev_name(&phydev->dev),
			  &ag71xx_phy_link_adjust_for_slave,
			  PHY_INTERFACE_MODE_MII);

		if (IS_ERR(ags->phy_dev)){
			printk(KERN_ERR "%s: could not connect to PHY\n",
					 ags->dev->name);
			return;
		}
	 	phydev->supported &= PHY_BASIC_FEATURES;
		phydev->advertising = phydev->supported;
		printk(KERN_DEBUG "%s:connected to PHY at %s [uid=%08x, driver=%s]\n",
					 ags->dev->name, dev_name(&phydev->dev), phydev->phy_id, phydev->drv->name);
	}
}

int ag71xx_phy_connect(struct ag71xx *ag)
{
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);

	if (pdata->mii_bus_dev == NULL ||
	    pdata->mii_bus_dev->bus == NULL )
		return ag71xx_phy_connect_fixed(ag);

	ag->mii_bus = dev_to_mii_bus(pdata->mii_bus_dev);
	if (ag->mii_bus == NULL) {
		dev_err(&ag->pdev->dev, "unable to find MII bus on device '%s'\n",
			   dev_name(pdata->mii_bus_dev));
		return -ENODEV;
	}

	/* Reset the mdio bus explicitly */
	if (ag->mii_bus->reset) {
		mutex_lock(&ag->mii_bus->mdio_lock);
		ag->mii_bus->reset(ag->mii_bus);
		mutex_unlock(&ag->mii_bus->mdio_lock);
	}

	if (pdata->switch_data)
		return ag71xx_ar7240_init(ag); //initial state для LAN портов свитча

	if (pdata->phy_mask)
		return ag71xx_phy_connect_multi(ag); //а вот эта ф-я на последок сделает initial state для WAN порта!

	return ag71xx_phy_connect_fixed(ag);
}

void ag71xx_phy_disconnect(struct ag71xx *ag)
{
	struct ag71xx_platform_data *pdata = ag71xx_get_pdata(ag);

	if (pdata->switch_data)
		ag71xx_ar7240_cleanup(ag);
	else if (ag->phy_dev)
		phy_disconnect(ag->phy_dev);
}
