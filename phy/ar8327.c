/*
 * ar8327.c: AR8216 switch driver
 *
 * Copyright (C) 2009 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2011-2012 Gabor Juhos <juhosg@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/lockdep.h>
#include <linux/ar8216_platform.h>
#include <linux/workqueue.h>
#include <linux/of_device.h>
#include <linux/leds.h>
#include <linux/mdio.h>

#include "ar8216.h"
#include "ar8327.h"

extern const struct ar8xxx_mib_desc ar8236_mibs[39];
extern const struct switch_attr ar8xxx_sw_attr_vlan[1];

static u32
ar8327_get_pad_cfg(struct ar8327_pad_cfg *cfg)
{
	u32 t;

	if (!cfg)
		return 0;

	t = 0;
	switch (cfg->mode) {
	case AR8327_PAD_NC:
		break;

	case AR8327_PAD_MAC2MAC_MII:
		t = AR8327_PAD_MAC_MII_EN;
		if (cfg->rxclk_sel)
			t |= AR8327_PAD_MAC_MII_RXCLK_SEL;
		if (cfg->txclk_sel)
			t |= AR8327_PAD_MAC_MII_TXCLK_SEL;
		break;

	case AR8327_PAD_MAC2MAC_GMII:
		t = AR8327_PAD_MAC_GMII_EN;
		if (cfg->rxclk_sel)
			t |= AR8327_PAD_MAC_GMII_RXCLK_SEL;
		if (cfg->txclk_sel)
			t |= AR8327_PAD_MAC_GMII_TXCLK_SEL;
		break;

	case AR8327_PAD_MAC_SGMII:
		t = AR8327_PAD_SGMII_EN;

		/*
		 * WAR for the QUalcomm Atheros AP136 board.
		 * It seems that RGMII TX/RX delay settings needs to be
		 * applied for SGMII mode as well, The ethernet is not
		 * reliable without this.
		 */
		t |= cfg->txclk_delay_sel << AR8327_PAD_RGMII_TXCLK_DELAY_SEL_S;
		t |= cfg->rxclk_delay_sel << AR8327_PAD_RGMII_RXCLK_DELAY_SEL_S;
		if (cfg->rxclk_delay_en)
			t |= AR8327_PAD_RGMII_RXCLK_DELAY_EN;
		if (cfg->txclk_delay_en)
			t |= AR8327_PAD_RGMII_TXCLK_DELAY_EN;

		if (cfg->sgmii_delay_en)
			t |= AR8327_PAD_SGMII_DELAY_EN;

		break;

	case AR8327_PAD_MAC2PHY_MII:
		t = AR8327_PAD_PHY_MII_EN;
		if (cfg->rxclk_sel)
			t |= AR8327_PAD_PHY_MII_RXCLK_SEL;
		if (cfg->txclk_sel)
			t |= AR8327_PAD_PHY_MII_TXCLK_SEL;
		break;

	case AR8327_PAD_MAC2PHY_GMII:
		t = AR8327_PAD_PHY_GMII_EN;
		if (cfg->pipe_rxclk_sel)
			t |= AR8327_PAD_PHY_GMII_PIPE_RXCLK_SEL;
		if (cfg->rxclk_sel)
			t |= AR8327_PAD_PHY_GMII_RXCLK_SEL;
		if (cfg->txclk_sel)
			t |= AR8327_PAD_PHY_GMII_TXCLK_SEL;
		break;

	case AR8327_PAD_MAC_RGMII:
		t = AR8327_PAD_RGMII_EN;
		t |= cfg->txclk_delay_sel << AR8327_PAD_RGMII_TXCLK_DELAY_SEL_S;
		t |= cfg->rxclk_delay_sel << AR8327_PAD_RGMII_RXCLK_DELAY_SEL_S;
		if (cfg->rxclk_delay_en)
			t |= AR8327_PAD_RGMII_RXCLK_DELAY_EN;
		if (cfg->txclk_delay_en)
			t |= AR8327_PAD_RGMII_TXCLK_DELAY_EN;
		break;

	case AR8327_PAD_PHY_GMII:
		t = AR8327_PAD_PHYX_GMII_EN;
		break;

	case AR8327_PAD_PHY_RGMII:
		t = AR8327_PAD_PHYX_RGMII_EN;
		break;

	case AR8327_PAD_PHY_MII:
		t = AR8327_PAD_PHYX_MII_EN;
		break;
	}

	return t;
}

static void
ar8327_phy_fixup(struct ar8xxx_priv *priv, int phy)
{
	switch (priv->chip_rev) {
	case 1:
		/* For 100M waveform */
		ar8xxx_phy_dbg_write(priv, phy, 0, 0x02ea);
		/* Turn on Gigabit clock */
		ar8xxx_phy_dbg_write(priv, phy, 0x3d, 0x68a0);
		break;

	case 2:
		ar8xxx_phy_mmd_write(priv, phy, 0x7, 0x3c, 0x0);
		/* fallthrough */
	case 4:
		ar8xxx_phy_mmd_write(priv, phy, 0x3, 0x800d, 0x803f);
		ar8xxx_phy_dbg_write(priv, phy, 0x3d, 0x6860);
		ar8xxx_phy_dbg_write(priv, phy, 0x5, 0x2c46);
		ar8xxx_phy_dbg_write(priv, phy, 0x3c, 0x6000);
		break;
	}
}

static u32
ar8327_get_port_init_status(struct ar8327_port_cfg *cfg)
{
	u32 t;

	if (!cfg->force_link)
		return AR8216_PORT_STATUS_LINK_AUTO;

	t = AR8216_PORT_STATUS_TXMAC | AR8216_PORT_STATUS_RXMAC;
	t |= cfg->duplex ? AR8216_PORT_STATUS_DUPLEX : 0;
	t |= cfg->rxpause ? AR8216_PORT_STATUS_RXFLOW : 0;
	t |= cfg->txpause ? AR8216_PORT_STATUS_TXFLOW : 0;

	switch (cfg->speed) {
	case AR8327_PORT_SPEED_10:
		t |= AR8216_PORT_SPEED_10M;
		break;
	case AR8327_PORT_SPEED_100:
		t |= AR8216_PORT_SPEED_100M;
		break;
	case AR8327_PORT_SPEED_1000:
		t |= AR8216_PORT_SPEED_1000M;
		break;
	}

	return t;
}

#define AR8327_LED_ENTRY(_num, _reg, _shift) \
	[_num] = { .reg = (_reg), .shift = (_shift) }

static const struct ar8327_led_entry
ar8327_led_map[AR8327_NUM_LEDS] = {
	AR8327_LED_ENTRY(AR8327_LED_PHY0_0, 0, 14),
	AR8327_LED_ENTRY(AR8327_LED_PHY0_1, 1, 14),
	AR8327_LED_ENTRY(AR8327_LED_PHY0_2, 2, 14),

	AR8327_LED_ENTRY(AR8327_LED_PHY1_0, 3, 8),
	AR8327_LED_ENTRY(AR8327_LED_PHY1_1, 3, 10),
	AR8327_LED_ENTRY(AR8327_LED_PHY1_2, 3, 12),

	AR8327_LED_ENTRY(AR8327_LED_PHY2_0, 3, 14),
	AR8327_LED_ENTRY(AR8327_LED_PHY2_1, 3, 16),
	AR8327_LED_ENTRY(AR8327_LED_PHY2_2, 3, 18),

	AR8327_LED_ENTRY(AR8327_LED_PHY3_0, 3, 20),
	AR8327_LED_ENTRY(AR8327_LED_PHY3_1, 3, 22),
	AR8327_LED_ENTRY(AR8327_LED_PHY3_2, 3, 24),

	AR8327_LED_ENTRY(AR8327_LED_PHY4_0, 0, 30),
	AR8327_LED_ENTRY(AR8327_LED_PHY4_1, 1, 30),
	AR8327_LED_ENTRY(AR8327_LED_PHY4_2, 2, 30),
};

static void
ar8327_set_led_pattern(struct ar8xxx_priv *priv, unsigned int led_num,
		       enum ar8327_led_pattern pattern)
{
	const struct ar8327_led_entry *entry;

	entry = &ar8327_led_map[led_num];
	ar8xxx_rmw(priv, AR8327_REG_LED_CTRL(entry->reg),
		   (3 << entry->shift), pattern << entry->shift);
}

static void
ar8327_led_work_func(struct work_struct *work)
{
	struct ar8327_led *aled;
	u8 pattern;

	aled = container_of(work, struct ar8327_led, led_work);

	pattern = aled->pattern;

	ar8327_set_led_pattern(aled->sw_priv, aled->led_num,
			       pattern);
}

static void
ar8327_led_schedule_change(struct ar8327_led *aled, u8 pattern)
{
	if (aled->pattern == pattern)
		return;

	aled->pattern = pattern;
	schedule_work(&aled->led_work);
}

static inline struct ar8327_led *
led_cdev_to_ar8327_led(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct ar8327_led, cdev);
}

static int
ar8327_led_blink_set(struct led_classdev *led_cdev,
		     unsigned long *delay_on,
		     unsigned long *delay_off)
{
	struct ar8327_led *aled = led_cdev_to_ar8327_led(led_cdev);

	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 125;
		*delay_off = 125;
	}

	if (*delay_on != 125 || *delay_off != 125) {
		/*
		 * The hardware only supports blinking at 4Hz. Fall back
		 * to software implementation in other cases.
		 */
		return -EINVAL;
	}

	spin_lock(&aled->lock);

	aled->enable_hw_mode = false;
	ar8327_led_schedule_change(aled, AR8327_LED_PATTERN_BLINK);

	spin_unlock(&aled->lock);

	return 0;
}

static void
ar8327_led_set_brightness(struct led_classdev *led_cdev,
			  enum led_brightness brightness)
{
	struct ar8327_led *aled = led_cdev_to_ar8327_led(led_cdev);
	u8 pattern;
	bool active;

	active = (brightness != LED_OFF);
	active ^= aled->active_low;

	pattern = (active) ? AR8327_LED_PATTERN_ON :
			     AR8327_LED_PATTERN_OFF;

	spin_lock(&aled->lock);

	aled->enable_hw_mode = false;
	ar8327_led_schedule_change(aled, pattern);

	spin_unlock(&aled->lock);
}

static ssize_t
ar8327_led_enable_hw_mode_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ar8327_led *aled = led_cdev_to_ar8327_led(led_cdev);
	ssize_t ret = 0;

	ret += scnprintf(buf, PAGE_SIZE, "%d\n", aled->enable_hw_mode);

	return ret;
}

static ssize_t
ar8327_led_enable_hw_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t size)
{
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ar8327_led *aled = led_cdev_to_ar8327_led(led_cdev);
	u8 pattern;
	u8 value;
	int ret;

	ret = kstrtou8(buf, 10, &value);
	if (ret < 0)
		return -EINVAL;

	spin_lock(&aled->lock);

	aled->enable_hw_mode = !!value;
	if (aled->enable_hw_mode)
		pattern = AR8327_LED_PATTERN_RULE;
	else
		pattern = AR8327_LED_PATTERN_OFF;

	ar8327_led_schedule_change(aled, pattern);

	spin_unlock(&aled->lock);

	return size;
}

static DEVICE_ATTR(enable_hw_mode,  S_IRUGO | S_IWUSR,
		   ar8327_led_enable_hw_mode_show,
		   ar8327_led_enable_hw_mode_store);

static int
ar8327_led_register(struct ar8327_led *aled)
{
	int ret;

	ret = led_classdev_register(NULL, &aled->cdev);
	if (ret < 0)
		return ret;

	if (aled->mode == AR8327_LED_MODE_HW) {
		ret = device_create_file(aled->cdev.dev,
					 &dev_attr_enable_hw_mode);
		if (ret)
			goto err_unregister;
	}

	return 0;

err_unregister:
	led_classdev_unregister(&aled->cdev);
	return ret;
}

static void
ar8327_led_unregister(struct ar8327_led *aled)
{
	if (aled->mode == AR8327_LED_MODE_HW)
		device_remove_file(aled->cdev.dev, &dev_attr_enable_hw_mode);

	led_classdev_unregister(&aled->cdev);
	cancel_work_sync(&aled->led_work);
}

static int
ar8327_led_create(struct ar8xxx_priv *priv,
		  const struct ar8327_led_info *led_info)
{
	struct ar8327_data *data = priv->chip_data;
	struct ar8327_led *aled;
	int ret;

	if (!IS_ENABLED(CONFIG_AR8216_PHY_LEDS))
		return 0;

	if (!led_info->name)
		return -EINVAL;

	if (led_info->led_num >= AR8327_NUM_LEDS)
		return -EINVAL;

	aled = kzalloc(sizeof(*aled) + strlen(led_info->name) + 1,
		       GFP_KERNEL);
	if (!aled)
		return -ENOMEM;

	aled->sw_priv = priv;
	aled->led_num = led_info->led_num;
	aled->active_low = led_info->active_low;
	aled->mode = led_info->mode;

	if (aled->mode == AR8327_LED_MODE_HW)
		aled->enable_hw_mode = true;

	aled->name = (char *)(aled + 1);
	strcpy(aled->name, led_info->name);

	aled->cdev.name = aled->name;
	aled->cdev.brightness_set = ar8327_led_set_brightness;
	aled->cdev.blink_set = ar8327_led_blink_set;
	aled->cdev.default_trigger = led_info->default_trigger;

	spin_lock_init(&aled->lock);
	mutex_init(&aled->mutex);
	INIT_WORK(&aled->led_work, ar8327_led_work_func);

	ret = ar8327_led_register(aled);
	if (ret)
		goto err_free;

	data->leds[data->num_leds++] = aled;

	return 0;

err_free:
	kfree(aled);
	return ret;
}

static void
ar8327_led_destroy(struct ar8327_led *aled)
{
	ar8327_led_unregister(aled);
	kfree(aled);
}

static void
ar8327_leds_init(struct ar8xxx_priv *priv)
{
	struct ar8327_data *data = priv->chip_data;
	unsigned i;

	if (!IS_ENABLED(CONFIG_AR8216_PHY_LEDS))
		return;

	for (i = 0; i < data->num_leds; i++) {
		struct ar8327_led *aled;

		aled = data->leds[i];

		if (aled->enable_hw_mode)
			aled->pattern = AR8327_LED_PATTERN_RULE;
		else
			aled->pattern = AR8327_LED_PATTERN_OFF;

		ar8327_set_led_pattern(priv, aled->led_num, aled->pattern);
	}
}

static void
ar8327_leds_cleanup(struct ar8xxx_priv *priv)
{
	struct ar8327_data *data = priv->chip_data;
	unsigned i;

	if (!IS_ENABLED(CONFIG_AR8216_PHY_LEDS))
		return;

	for (i = 0; i < data->num_leds; i++) {
		struct ar8327_led *aled;

		aled = data->leds[i];
		ar8327_led_destroy(aled);
	}

	kfree(data->leds);
}

static int
ar8327_hw_config_pdata(struct ar8xxx_priv *priv,
		       struct ar8327_platform_data *pdata)
{
	struct ar8327_led_cfg *led_cfg;
	struct ar8327_data *data = priv->chip_data;
	u32 pos, new_pos;
	u32 t;

	if (!pdata)
		return -EINVAL;

	priv->get_port_link = pdata->get_port_link;
	priv->port6_custom_set_power_state = pdata->port6_custom_set_power_state;

	data->port0_status = ar8327_get_port_init_status(&pdata->port0_cfg);
	data->port6_status = ar8327_get_port_init_status(&pdata->port6_cfg);

	t = ar8327_get_pad_cfg(pdata->pad0_cfg);
	if (chip_is_ar8337(priv) && !pdata->pad0_cfg->mac06_exchange_dis)
	    t |= AR8337_PAD_MAC06_EXCHANGE_EN;
	ar8xxx_write(priv, AR8327_REG_PAD0_MODE, t);

	t = ar8327_get_pad_cfg(pdata->pad5_cfg);
	ar8xxx_write(priv, AR8327_REG_PAD5_MODE, t);
	t = ar8327_get_pad_cfg(pdata->pad6_cfg);
	ar8xxx_write(priv, AR8327_REG_PAD6_MODE, t);

	pos = ar8xxx_read(priv, AR8327_REG_POWER_ON_STRIP);
	new_pos = pos;

	led_cfg = pdata->led_cfg;
	if (led_cfg) {
		if (led_cfg->open_drain)
			new_pos |= AR8327_POWER_ON_STRIP_LED_OPEN_EN;
		else
			new_pos &= ~AR8327_POWER_ON_STRIP_LED_OPEN_EN;

		ar8xxx_write(priv, AR8327_REG_LED_CTRL0, led_cfg->led_ctrl0);
		ar8xxx_write(priv, AR8327_REG_LED_CTRL1, led_cfg->led_ctrl1);
		ar8xxx_write(priv, AR8327_REG_LED_CTRL2, led_cfg->led_ctrl2);
		ar8xxx_write(priv, AR8327_REG_LED_CTRL3, led_cfg->led_ctrl3);

		if (new_pos != pos)
			new_pos |= AR8327_POWER_ON_STRIP_POWER_ON_SEL;
	}

	if (pdata->sgmii_cfg) {
		t = pdata->sgmii_cfg->sgmii_ctrl;
		if (priv->chip_rev == 1)
			t |= AR8327_SGMII_CTRL_EN_PLL |
			     AR8327_SGMII_CTRL_EN_RX |
			     AR8327_SGMII_CTRL_EN_TX;
		else
			t &= ~(AR8327_SGMII_CTRL_EN_PLL |
			       AR8327_SGMII_CTRL_EN_RX |
			       AR8327_SGMII_CTRL_EN_TX);

		/* printk(KERN_DEBUG "sgmii_ctrl = 0x%x, serdes_aen = %d, chip_rev = %d\n",
			t, pdata->sgmii_cfg->serdes_aen, priv->chip_rev); */

		ar8xxx_write(priv, AR8327_REG_SGMII_CTRL, t);

		if (pdata->sgmii_cfg->serdes_aen)
			new_pos &= ~AR8327_POWER_ON_STRIP_SERDES_AEN;
		else
			new_pos |= AR8327_POWER_ON_STRIP_SERDES_AEN;
	}

	ar8xxx_write(priv, AR8327_REG_POWER_ON_STRIP, new_pos);

	if (pdata->leds && pdata->num_leds) {
		int i;

		data->leds = kzalloc(pdata->num_leds * sizeof(void *),
				     GFP_KERNEL);
		if (!data->leds)
			return -ENOMEM;

		for (i = 0; i < pdata->num_leds; i++)
			ar8327_led_create(priv, &pdata->leds[i]);
	}

	return 0;
}

#ifdef CONFIG_OF
static int
ar8327_hw_config_of(struct ar8xxx_priv *priv, struct device_node *np)
{
	struct ar8327_data *data = priv->chip_data;
	const __be32 *paddr;
	int len;
	int i;

	paddr = of_get_property(np, "qca,ar8327-initvals", &len);
	if (!paddr || len < (2 * sizeof(*paddr)))
		return -EINVAL;

	len /= sizeof(*paddr);

	for (i = 0; i < len - 1; i += 2) {
		u32 reg;
		u32 val;

		reg = be32_to_cpup(paddr + i);
		val = be32_to_cpup(paddr + i + 1);

		switch (reg) {
		case AR8327_REG_PORT_STATUS(0):
			data->port0_status = val;
			break;
		case AR8327_REG_PORT_STATUS(6):
			data->port6_status = val;
			break;
		default:
			ar8xxx_write(priv, reg, val);
			break;
		}
	}

	return 0;
}
#else
static inline int
ar8327_hw_config_of(struct ar8xxx_priv *priv, struct device_node *np)
{
	return -EINVAL;
}
#endif

static int
ar8327_hw_init(struct ar8xxx_priv *priv)
{
	int ret;

	priv->chip_data = kzalloc(sizeof(struct ar8327_data), GFP_KERNEL);
	if (!priv->chip_data)
		return -ENOMEM;

	if (priv->phy->dev.of_node)
		ret = ar8327_hw_config_of(priv, priv->phy->dev.of_node);
	else
		ret = ar8327_hw_config_pdata(priv,
					     priv->phy->dev.platform_data);
	if (ret)
		return ret;

	ar8327_leds_init(priv);

	ar8xxx_phy_init(priv);

	return 0;
}

static void
ar8327_cleanup(struct ar8xxx_priv *priv)
{
	ar8327_leds_cleanup(priv);
}

static void
ar8327_init_globals(struct ar8xxx_priv *priv)
{
	struct ar8327_data *data = priv->chip_data;
	u32 t;
	int i;

	//printk(KERN_ERR "%s: iface_mode = %d\n", __func__, priv->iface_mode);

	/* enable CPU port and disable mirror port */
	t = AR8327_FWD_CTRL0_CPU_PORT_EN |
	    AR8327_FWD_CTRL0_MIRROR_PORT;
	ar8xxx_write(priv, AR8327_REG_FWD_CTRL0, t);

	/* forward multicast and broadcast frames to CPU */
	t = (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_UC_FLOOD_S) |
	    (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_MC_FLOOD_S) |
	    (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_BC_FLOOD_S);
	ar8xxx_write(priv, AR8327_REG_FWD_CTRL1, t);

	/* enable jumbo frames */
	ar8xxx_rmw(priv, AR8327_REG_MAX_FRAME_SIZE,
		   AR8327_MAX_FRAME_SIZE_MTU, 9018 + 8 + 2);

	/* Enable MIB counters */
	ar8xxx_reg_set(priv, AR8327_REG_MODULE_EN,
		       AR8327_MODULE_EN_MIB);

	/* Disable EEE on all phy's due to stability issues */
	for (i = 0; i < AR8XXX_NUM_PHYS; i++)
		data->eee[i] = false;
}

void ar8327_set_port_state(struct switch_dev *, unsigned port, int);

static void
ar8327_init_port(struct ar8xxx_priv *priv, int port)
{
	struct ar8327_data *data = priv->chip_data;
	u32 t;

	//printk(KERN_ERR "%s: iface_mode = %d, port = %d\n", __func__, priv->iface_mode, port);

	if (port == AR8216_PORT_CPU)
		t = data->port0_status;
	else if (port == 6)
		t = data->port6_status;
	else
		t = AR8216_PORT_STATUS_LINK_AUTO;

	if (port != AR8216_PORT_CPU && port != 6) {
		/*hw limitation:if configure mac when there is traffic,
		port MAC may work abnormal. Need disable lan&wan mac at fisrt*/
		ar8xxx_write(priv, AR8327_REG_PORT_STATUS(port), 0);
		msleep(100);
		t |= AR8216_PORT_STATUS_FLOW_CONTROL;
		ar8xxx_write(priv, AR8327_REG_PORT_STATUS(port), t);
	} else {
		ar8xxx_write(priv, AR8327_REG_PORT_STATUS(port), t);
	}

	ar8xxx_write(priv, AR8327_REG_PORT_HEADER(port), 0);
	ar8xxx_write(priv, AR8327_REG_PORT_VLAN0(port), 0);

	t = AR8327_PORT_VLAN1_OUT_MODE_UNTOUCH << AR8327_PORT_VLAN1_OUT_MODE_S;
	ar8xxx_write(priv, AR8327_REG_PORT_VLAN1(port), t);

	if(!priv->iface_mode)
		t = AR8327_PORT_LOOKUP_LEARN;
	else
		t = 0;
	t |= AR8216_PORT_STATE_FORWARD << AR8327_PORT_LOOKUP_STATE_S;
	ar8xxx_write(priv, AR8327_REG_PORT_LOOKUP(port), t);
	/* порт в этом случае поднимет уже slave_dev_open
		 а пока что порт будет DISABLED и PHY is off. */
	if(priv->iface_mode)
		ar8327_set_port_state(&priv->dev, port, 0);
}

static u32
ar8327_read_port_status(struct ar8xxx_priv *priv, int port)
{
	u32 t;

	t = ar8xxx_read(priv, AR8327_REG_PORT_STATUS(port));
	/* map the flow control autoneg result bits to the flow control bits
	 * used in forced mode to allow ar8216_read_port_link detect
	 * flow control properly if autoneg is used
	 */
	if (t & AR8216_PORT_STATUS_LINK_UP &&
	    t & AR8216_PORT_STATUS_LINK_AUTO) {
		t &= ~(AR8216_PORT_STATUS_TXFLOW | AR8216_PORT_STATUS_RXFLOW);
		if (t & AR8327_PORT_STATUS_TXFLOW_AUTO)
			t |= AR8216_PORT_STATUS_TXFLOW;
		if (t & AR8327_PORT_STATUS_RXFLOW_AUTO)
			t |= AR8216_PORT_STATUS_RXFLOW;
	}

	return t;
}

static u32
ar8327_read_port_eee_status(struct ar8xxx_priv *priv, int port)
{
	int phy;
	u16 t;

	if (port >= priv->dev.ports)
		return 0;

	if (port == 0 || port == 6)
		return 0;

	phy = port - 1;

	/* EEE Ability Auto-negotiation Result */
	t = ar8xxx_phy_mmd_read(priv, phy, 0x7, 0x8000);

	return mmd_eee_adv_to_ethtool_adv_t(t);
}

static int
ar8327_atu_flush(struct ar8xxx_priv *priv)
{
	int ret;

	ret = ar8216_wait_bit(priv, AR8327_REG_ATU_FUNC,
			      AR8327_ATU_FUNC_BUSY, 0);
	if (!ret)
		ar8xxx_write(priv, AR8327_REG_ATU_FUNC,
			     AR8327_ATU_FUNC_OP_FLUSH |
			     AR8327_ATU_FUNC_BUSY);

	return ret;
}

static int
ar8327_atu_flush_port(struct ar8xxx_priv *priv, int port)
{
	u32 t;
	int ret;

	ret = ar8216_wait_bit(priv, AR8327_REG_ATU_FUNC,
			      AR8327_ATU_FUNC_BUSY, 0);
	if (!ret) {
		t = (port << AR8327_ATU_PORT_NUM_S);
		t |= AR8327_ATU_FUNC_OP_FLUSH_PORT;
		t |= AR8327_ATU_FUNC_BUSY;
		ar8xxx_write(priv, AR8327_REG_ATU_FUNC, t);
	}

	return ret;
}

static int
ar8327_get_port_igmp(struct ar8xxx_priv *priv, int port)
{
	u32 fwd_ctrl, frame_ack;

	fwd_ctrl = (BIT(port) << AR8327_FWD_CTRL1_IGMP_S);
	frame_ack = ((AR8327_FRAME_ACK_CTRL_IGMP_MLD |
		      AR8327_FRAME_ACK_CTRL_IGMP_JOIN |
		      AR8327_FRAME_ACK_CTRL_IGMP_LEAVE) <<
		     AR8327_FRAME_ACK_CTRL_S(port));

	return (ar8xxx_read(priv, AR8327_REG_FWD_CTRL1) &
			fwd_ctrl) == fwd_ctrl &&
		(ar8xxx_read(priv, AR8327_REG_FRAME_ACK_CTRL(port)) &
			frame_ack) == frame_ack;
}

static void
ar8327_set_port_igmp(struct ar8xxx_priv *priv, int port, int enable)
{
	int reg_frame_ack = AR8327_REG_FRAME_ACK_CTRL(port);
	u32 val_frame_ack = (AR8327_FRAME_ACK_CTRL_IGMP_MLD |
			  AR8327_FRAME_ACK_CTRL_IGMP_JOIN |
			  AR8327_FRAME_ACK_CTRL_IGMP_LEAVE) <<
			 AR8327_FRAME_ACK_CTRL_S(port);

	if (enable) {
		ar8xxx_rmw(priv, AR8327_REG_FWD_CTRL1,
			   BIT(port) << AR8327_FWD_CTRL1_MC_FLOOD_S,
			   BIT(port) << AR8327_FWD_CTRL1_IGMP_S);
		ar8xxx_reg_set(priv, reg_frame_ack, val_frame_ack);
	} else {
		ar8xxx_rmw(priv, AR8327_REG_FWD_CTRL1,
			   BIT(port) << AR8327_FWD_CTRL1_IGMP_S,
			   BIT(port) << AR8327_FWD_CTRL1_MC_FLOOD_S);
		ar8xxx_reg_clear(priv, reg_frame_ack, val_frame_ack);
	}
}

static void
ar8327_vtu_op(struct ar8xxx_priv *priv, u32 op, u32 val)
{
	if (ar8216_wait_bit(priv, AR8327_REG_VTU_FUNC1,
			    AR8327_VTU_FUNC1_BUSY, 0))
		return;

	if ((op & AR8327_VTU_FUNC1_OP) == AR8327_VTU_FUNC1_OP_LOAD)
		ar8xxx_write(priv, AR8327_REG_VTU_FUNC0, val);

	op |= AR8327_VTU_FUNC1_BUSY;
	ar8xxx_write(priv, AR8327_REG_VTU_FUNC1, op);
}

static void
ar8327_vtu_flush(struct ar8xxx_priv *priv)
{
	ar8327_vtu_op(priv, AR8327_VTU_FUNC1_OP_FLUSH, 0);
}

static void
ar8327_vtu_load_vlan(struct ar8xxx_priv *priv, u32 vid, u32 port_mask)
{
	u32 op;
	u32 val;
	int i;

	op = AR8327_VTU_FUNC1_OP_LOAD | (vid << AR8327_VTU_FUNC1_VID_S);
	val = AR8327_VTU_FUNC0_VALID | AR8327_VTU_FUNC0_IVL;
	for (i = 0; i < AR8327_NUM_PORTS; i++) {
		u32 mode;

		if ((port_mask & BIT(i)) == 0)
			mode = AR8327_VTU_FUNC0_EG_MODE_NOT;
		else if (priv->vlan == 0)
			mode = AR8327_VTU_FUNC0_EG_MODE_KEEP;
		else if ((priv->vlan_tagged & BIT(i)) || (priv->vlan_id[priv->pvid[i]] != vid))
			mode = AR8327_VTU_FUNC0_EG_MODE_TAG;
		else
			mode = AR8327_VTU_FUNC0_EG_MODE_UNTAG;

		val |= mode << AR8327_VTU_FUNC0_EG_MODE_S(i);
	}
	ar8327_vtu_op(priv, op, val);
}

static void ar8327_setup_port_helper(struct switch_dev *, int);
static void
ar8327_setup_port(struct ar8xxx_priv *priv, int port, u32 members)
{
	u32 t;
	u32 egress, ingress;
	u32 pvid = priv->vlan_id[priv->pvid[port]];

	/* printk(KERN_DEBUG "%s: port = %d, members = 0x%x",
			 __func__, port, members); */

	if (priv->vlan) {
		egress = AR8327_PORT_VLAN1_OUT_MODE_UNMOD;
		ingress = AR8216_IN_SECURE;
	} else {
		egress = AR8327_PORT_VLAN1_OUT_MODE_UNTOUCH;
		ingress = AR8216_IN_PORT_ONLY;
	}

	t = pvid << AR8327_PORT_VLAN0_DEF_SVID_S;
	t |= pvid << AR8327_PORT_VLAN0_DEF_CVID_S;
	ar8xxx_write(priv, AR8327_REG_PORT_VLAN0(port), t);

	t = AR8327_PORT_VLAN1_PORT_VLAN_PROP;
	t |= egress << AR8327_PORT_VLAN1_OUT_MODE_S;
	ar8xxx_write(priv, AR8327_REG_PORT_VLAN1(port), t);

	t = members;
	if(!priv->iface_mode) //в режиме iface_mode НИКАКОГО learning!
		t |= AR8327_PORT_LOOKUP_LEARN;
	t |= ingress << AR8327_PORT_LOOKUP_IN_MODE_S;
	t |= AR8216_PORT_STATE_FORWARD << AR8327_PORT_LOOKUP_STATE_S;
	ar8xxx_write(priv, AR8327_REG_PORT_LOOKUP(port), t);
	/* Если iface_mode => врубаем Atheros headers для CPU порта */
	if(priv->iface_mode && port == AR8216_PORT_CPU)
		ar8xxx_write(priv, AR8327_REG_PORT_HEADER(port), 0xA);
	else
		ar8xxx_write(priv, AR8327_REG_PORT_HEADER(port), 0);
	ar8327_setup_port_helper(&priv->dev, port);
}

static int
ar8327_sw_get_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u8 ports = priv->vlan_table[val->port_vlan];
	int i;

	val->len = 0;
	for (i = 0; i < dev->ports; i++) {
		struct switch_port *p;

		if (!(ports & (1 << i)))
			continue;

		p = &val->value.ports[val->len++];
		p->id = i;
		if ((priv->vlan_tagged & (1 << i)) || (priv->pvid[i] != val->port_vlan))
			p->flags = (1 << SWITCH_PORT_FLAG_TAGGED);
		else
			p->flags = 0;
	}
	return 0;
}

static int
ar8327_sw_set_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u8 *vt = &priv->vlan_table[val->port_vlan];
	int i;

	*vt = 0;
	for (i = 0; i < val->len; i++) {
		struct switch_port *p = &val->value.ports[i];

		if (p->flags & (1 << SWITCH_PORT_FLAG_TAGGED)) {
			if (val->port_vlan == priv->pvid[p->id]) {
				priv->vlan_tagged |= (1 << p->id);
			}
		} else {
			priv->vlan_tagged &= ~(1 << p->id);
			priv->pvid[p->id] = val->port_vlan;
		}

		*vt |= 1 << p->id;
	}
	return 0;
}


/* возвращает указатель на mii_bus из swdev. используется во внешних
	 (например ag71xx) драйверах для вызова mdiobus_read/write. */
struct mii_bus *ar8327_get_mii_bus_from_swdev(struct switch_dev *dev){
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	return priv->mii_bus;
}


/* смещение ДВУХ битов 25:24 управления лампочкой для
	 регистра LED_CTRL3(0x5C) свитча AR8327 */
#define RB2011_NEW_SFP_PHY_ONOFF_S 24

/* включает/выключает питание на трансивере SFP порта устройства Mikrotik rb2011r5 */
static void rb2011_r5_set_port_power_state(struct ar8xxx_priv *priv, unsigned port, int state){
	u32 val;
	if(port != 6)
		return;
	val = ar8xxx_read(priv, AR8327_REG_LED_CTRL3);
	val &= ~(0x3 << RB2011_NEW_SFP_PHY_ONOFF_S); //онуливаем
	if(state)
		val |= 0x2 << RB2011_NEW_SFP_PHY_ONOFF_S; //включаем
	ar8xxx_write(priv, AR8327_REG_LED_CTRL3, val);
}

/* включает/выключает питание на трансивере указанного порта. умеет работать
	 как с обычными портами так и с нестандартными(например SFP в 2011r5). */
static void ar8327_set_port_power_state(struct ar8xxx_priv *priv, unsigned port, int state){
	u32 bmcr;
	struct mii_bus *bus = priv->mii_bus;
	if(!bus)
		return;
	if(port == 6 && priv->port6_custom_set_power_state > 0){
		if(priv->port6_custom_set_power_state == 20115)
			rb2011_r5_set_port_power_state(priv, port, state);
		return;
	}
	//printk(KERN_DEBUG "%s: port = %u, state = %d, bus = 0x%p\n", __func__, port, state, bus);
	bmcr = mdiobus_read(bus, port - 1, MII_BMCR);
	//printk(KERN_DEBUG "%s(%u, %d): bmcr do = 0x%x\n", __func__, port, state, bmcr);
	if(state){
		bmcr &= ~BMCR_PDOWN; //включаем питание на трансивер
		bmcr |= BMCR_RESET; //дальше обязательно ресет чтобы применить изменения
	}else{
		//отрубаем трансивер
		bmcr |= BMCR_PDOWN;
	}
	mdiobus_write(bus, port - 1, MII_BMCR, bmcr);
	//printk(KERN_DEBUG "%s(%u, %d): bmcr po = 0x%x\n", __func__, port, state, bmcr);
}

/* эта ф-я только для ar8327(и возможно для ar8216) !
	 Она аналогична ar7240_set_port_state в самом ag71xx.  */
void ar8327_set_port_state(struct switch_dev *dev, unsigned port, int state)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u32 ctrl;
	/* ЛОК! Никаких printk и return ! */
	mutex_lock(&priv->reg_mutex);
	ctrl = ar8xxx_read(priv, AR8327_REG_PORT_LOOKUP(port));
	ctrl &= ~AR8327_PORT_LOOKUP_STATE; //очистим биты PORT_LOOKUP_STATE
	if(state){
		ctrl |= AR8216_PORT_STATE_FORWARD << AR8327_PORT_LOOKUP_STATE_S;
	}else{
		//отрубаем свитч порт(запрет всего tx/rx трафика)
		ctrl |= AR8216_PORT_STATE_DISABLED << AR8327_PORT_LOOKUP_STATE_S;
	}
	ar8xxx_write(priv, AR8327_REG_PORT_LOOKUP(port), ctrl);
	//включаем или выключаем питание на трансивер
	ar8327_set_port_power_state(priv, port, state);
	/* UnLOCK */
	mutex_unlock(&priv->reg_mutex);
}

/* возвращает указатель на struct ag71xx через swdev */
static void *get_master_ag_from_swdev(struct switch_dev *dev)
{
	if(!dev->netdev)
		return NULL;
	return netdev_priv(dev->netdev);
}

/* проверялка номера порта для массива advertise */
static int port_check_for_advertise(struct switch_dev *dev, int port);
/* ф-я в коде ag71xx которую мы дергаем при установке параметра port link */
int cross_sw_set_port_link(void *, struct switch_dev *, int, struct switch_port_link *, u8);
/* ф-я в коде ag71xx которую дергает cross_sw_set_port_link и setup_port */
int ag71xx_cross_sw_adjust_port_link(void *, u32, struct switch_port_link *, u8);
/* ф-я вызываемая при установке: port x set link y */
static int ar8327_set_port_link(struct switch_dev *dev, int port,
			     struct switch_port_link *port_link)
{
	void *master_ag = get_master_ag_from_swdev(dev);
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u8 advertise = 0;
	if(!master_ag)
		return -EINVAL;
	if(!port_check_for_advertise(dev, port))
		advertise = priv->advertise[port];
	return cross_sw_set_port_link(master_ag, dev, port, port_link, advertise);
}

/* вспомогательная ф-я вызываемая из ar8327_setup_port. при ее вызове происходит
	 переприменение уже ранее установленных параметров скоростей и режимов порта. */
static void ar8327_setup_port_helper(struct switch_dev *dev, int port)
{
	void *master_ag = get_master_ag_from_swdev(dev);
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u8 advertise = 0;
	if(!master_ag)
		return;
	if(!port_check_for_advertise(dev, port))
		advertise = priv->advertise[port];

	ag71xx_cross_sw_adjust_port_link(master_ag, port, NULL, advertise);
}

/* вспомогательная ф-я вызываемая из ag71xx_cross_sw_adjust_port_link
	 для установки параметра link для 6-го порта нашего свитча. */
void ar8327_adjust_port_link(struct switch_dev *dev, u32 port,
struct switch_port_link *port_link, u8 advertise){
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	struct ar8327_data *data = priv->chip_data;
	struct ar8327_port_cfg cfg;
	u32 t;
	if(port != 6)
		return;
	/* 6-й порт у нас это как правило sfp и доступа через MII BMC регистры
	 к PHY у нас нет. Так что тут крутим что есть. */
	/* printk(KERN_INFO "%s: %d, port_link->aneg = %d, advertise = 0x%x, port_link->speed=%d\n",
		__func__, port, port_link->aneg, advertise, port_link->speed); */
	/* сформируем фейковый cfg для ф-и ar8327_get_port_init_status.
		 я это все делаю чтобы иметь возможность на горячую
		 посредством команды swconfig dev switch0 port 6 set link "duplex full speed 1000 autoneg off"
		 переключать порт в режим автонеготирации и обратно. в OpenWRT режим порта(автонеготинация есть
		 или нет) выбирается железно в mach-rb*.c файле. Но меня это не устраивает. Для SFP портов
		 бывает очень важно отключить неготинацию и железно установить скорость.
	*/
	if(port_link->aneg){
		cfg.force_link = 0;
	}else{
		cfg.force_link = 1;
	}
	cfg.duplex = !!port_link->duplex;
	switch(port_link->speed){
		case SWITCH_PORT_SPEED_10:
			cfg.speed = AR8327_PORT_SPEED_10;
			break;
		case SWITCH_PORT_SPEED_100:
			cfg.speed = AR8327_PORT_SPEED_100;
			break;
		case SWITCH_PORT_SPEED_1000:
			cfg.speed = AR8327_PORT_SPEED_1000;
			break;
		default:
			cfg.speed = 0;
	}
	cfg.txpause = 1; //tx и rx flow control пока оставляем влюченным.
	cfg.rxpause = 1; //в будущем можно будет приделать выключатель.
	//получаем значения битов
	t =	ar8327_get_port_init_status(&cfg);
	//printk(KERN_INFO "%s: 0x%x vs 0x%x\n", __func__, t, data->port6_status);
	if(t != data->port6_status){
		ar8xxx_write(priv, AR8327_REG_PORT_STATUS(port), t);
		//в этом драйвере статус 6-го порта сохраняется в data->port6_status !
		data->port6_status = t;
	}
}

#define OWL_DEBUG

#ifdef OWL_DEBUG
#include <linux/kallsyms.h>
typedef u32(AoWL_debug_cb)(void *, void *, void *, void *, u32, u32);
#endif

/* ф-я в коде ag71xx которую мы дергаем при установке параметра iface_mode == 1 */
int ag71xx_ext_sw_create_slave_devices(struct net_device *net_dev,
	struct switch_dev *sw_dev);

static int
ar8xxx_sw_set_iface_mode(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	bool iface_mode = !!val->value.i;
#ifdef OWL_DEBUG /* блок для глубокой отладки */
	if(val->value.i >= 222){
		AoWL_debug_cb *owl;
		printk(KERN_DEBUG "%s: iface_mode = DEBUG call !, priv = %p",
				 __func__, priv);
		owl = (void*)kallsyms_lookup_name("AoWL_debug_cb");
		if(owl)
			owl(dev, priv, priv->mii_bus, 0x0, 0x0, val->value.i);
		else
			printk(KERN_ERR "%s: Can't lookup AoWL_debug_cb !!!\n", __func__);
		return 0;
	}
#endif
	/* printk(KERN_DEBUG "%s: iface_mode = %s, prev_iface_mode = %s\n",
				 __func__, iface_mode ? "true" : "false",
				 priv->iface_mode ? "true" : "false"); */

	//задают тоже значение что уже было установлено ранее
	if(priv->iface_mode == iface_mode)
		return 0;
	//если пытаются на горячую отключить iface_mode
	if(priv->iface_mode && !iface_mode){
		printk(KERN_ERR "%s: I can't disarm iface_mode. Please reboot!\n", __func__);
		return -EINVAL;
	}

	priv->iface_mode = iface_mode;
	priv->vlan = false; //вланы должны быть выключены в этом режиме!
	if(iface_mode){
		struct net_device *ag71xx_net_dev = dev_get_by_name(&init_net, priv->ag71xx_dev_name);
		if(!ag71xx_net_dev){
			printk(KERN_ERR "%s: Can't find '%s' net device! Please enter it manually.\n",
				__func__, priv->ag71xx_dev_name);
			return -EINVAL;
		}
		/* это кусочек кода взят из register_switch. т.к. там делается вызов с netdev == NULL
			 то свитч получается без привязки к сетевому интерфейсу. здаесь мы это исправляем. */
		dev->netdev = ag71xx_net_dev;
		if(ag71xx_net_dev->name[0] != '\0')
			dev->alias = ag71xx_net_dev->name; //чтобы работало: swconfig dev eth0
		//создаем slave устройства
		ag71xx_ext_sw_create_slave_devices(ag71xx_net_dev, dev);
		dev_put(ag71xx_net_dev);
	}

	return 0;
}

static int
ar8xxx_sw_get_iface_mode(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	/* printk(KERN_DEBUG "%s: %d\n",
				 __func__, priv->iface_mode); */

	val->value.i = priv->iface_mode;
	return 0;
}

static int
ar8xxx_sw_set_ag71xx_ifname(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	priv->ag71xx_dev_name[IFNAMSIZ - 1] = '\0';
	strncpy(priv->ag71xx_dev_name, val->value.s, IFNAMSIZ - 1);
	return 0;
}

static int port_check_for_advertise(struct switch_dev *dev, int port){
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	if (port >= dev->ports)
		return -EINVAL;
	if (port == 0 || port == 6)
		return -EOPNOTSUPP;
	if(port >= sizeof(priv->advertise) / sizeof(priv->advertise[0]))
		return -EINVAL;
	return 0;
}

static int
ar8xxx_sw_get_ag71xx_ifname(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	val->value.s = priv->ag71xx_dev_name;
	return 0;
}


static int
ar8xxx_sw_set_advertise_speeds(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	int port = val->port_vlan;
	int ret;

	ret = port_check_for_advertise(dev, port);
	if(ret)
		return ret;

	priv->advertise[port] = val->value.i & 0xFF;
	return 0;
}

static int
ar8xxx_sw_get_advertise_speeds(struct switch_dev *dev, const struct switch_attr *attr,
		   struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	int port = val->port_vlan;
	int ret;

	ret = port_check_for_advertise(dev, port);
	if(ret)
		return ret;

	val->value.i = priv->advertise[port] & 0xFF;
	return 0;
}

static void
ar8327_set_mirror_regs(struct ar8xxx_priv *priv)
{
	int port;

	//printk(KERN_ERR "%s: set MIRROR_REGS !!!\n", __func__);
	/* reset all mirror registers */
	ar8xxx_rmw(priv, AR8327_REG_FWD_CTRL0,
		   AR8327_FWD_CTRL0_MIRROR_PORT,
		   (0xF << AR8327_FWD_CTRL0_MIRROR_PORT_S));
	for (port = 0; port < AR8327_NUM_PORTS; port++) {
		ar8xxx_reg_clear(priv, AR8327_REG_PORT_LOOKUP(port),
			   AR8327_PORT_LOOKUP_ING_MIRROR_EN);

		ar8xxx_reg_clear(priv, AR8327_REG_PORT_HOL_CTRL1(port),
			   AR8327_PORT_HOL_CTRL1_EG_MIRROR_EN);
	}

	/* now enable mirroring if necessary */
	if (priv->source_port >= AR8327_NUM_PORTS ||
	    priv->monitor_port >= AR8327_NUM_PORTS ||
	    priv->source_port == priv->monitor_port) {
		return;
	}

	ar8xxx_rmw(priv, AR8327_REG_FWD_CTRL0,
		   AR8327_FWD_CTRL0_MIRROR_PORT,
		   (priv->monitor_port << AR8327_FWD_CTRL0_MIRROR_PORT_S));

	if (priv->mirror_rx)
		ar8xxx_reg_set(priv, AR8327_REG_PORT_LOOKUP(priv->source_port),
			   AR8327_PORT_LOOKUP_ING_MIRROR_EN);

	if (priv->mirror_tx)
		ar8xxx_reg_set(priv, AR8327_REG_PORT_HOL_CTRL1(priv->source_port),
			   AR8327_PORT_HOL_CTRL1_EG_MIRROR_EN);
}

static int
ar8327_sw_set_eee(struct switch_dev *dev,
		  const struct switch_attr *attr,
		  struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	struct ar8327_data *data = priv->chip_data;
	int port = val->port_vlan;
	int phy;

	if (port >= dev->ports)
		return -EINVAL;
	if (port == 0 || port == 6)
		return -EOPNOTSUPP;

	phy = port - 1;

	data->eee[phy] = !!(val->value.i);

	return 0;
}

static int
ar8327_sw_get_eee(struct switch_dev *dev,
		  const struct switch_attr *attr,
		  struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	const struct ar8327_data *data = priv->chip_data;
	int port = val->port_vlan;
	int phy;

	if (port >= dev->ports)
		return -EINVAL;
	if (port == 0 || port == 6)
		return -EOPNOTSUPP;

	phy = port - 1;

	val->value.i = data->eee[phy];

	return 0;
}

#if 0
static void
ar8327_wait_atu_ready(struct ar8xxx_priv *priv, u16 r2, u16 r1)
{
	int timeout = 20;

	while (ar8xxx_mii_read32(priv, r2, r1) & AR8327_ATU_FUNC_BUSY && --timeout)
                udelay(10);

	if (!timeout)
		pr_err("ar8327: timeout waiting for atu to become ready\n");
}

static void ar8327_get_arl_entry(struct ar8xxx_priv *priv,
				 struct arl_entry *a, u32 *status, enum arl_op op)
{
	struct mii_bus *bus = priv->mii_bus;
	u16 r2, page;
	u16 r1_data0, r1_data1, r1_data2, r1_func;
	u32 t, val0, val1, val2;
	int i;

	split_addr(AR8327_REG_ATU_DATA0, &r1_data0, &r2, &page);
	r2 |= 0x10;

	r1_data1 = (AR8327_REG_ATU_DATA1 >> 1) & 0x1e;
	r1_data2 = (AR8327_REG_ATU_DATA2 >> 1) & 0x1e;
	r1_func  = (AR8327_REG_ATU_FUNC >> 1) & 0x1e;

	switch (op) {
	case AR8XXX_ARL_INITIALIZE:
		/* all ATU registers are on the same page
		* therefore set page only once
		*/
		bus->write(bus, 0x18, 0, page);
		wait_for_page_switch();

		ar8327_wait_atu_ready(priv, r2, r1_func);

		ar8xxx_mii_write32(priv, r2, r1_data0, 0);
		ar8xxx_mii_write32(priv, r2, r1_data1, 0);
		ar8xxx_mii_write32(priv, r2, r1_data2, 0);
		break;
	case AR8XXX_ARL_GET_NEXT:
		ar8xxx_mii_write32(priv, r2, r1_func,
				   AR8327_ATU_FUNC_OP_GET_NEXT |
				   AR8327_ATU_FUNC_BUSY);
		ar8327_wait_atu_ready(priv, r2, r1_func);

		val0 = ar8xxx_mii_read32(priv, r2, r1_data0);
		val1 = ar8xxx_mii_read32(priv, r2, r1_data1);
		val2 = ar8xxx_mii_read32(priv, r2, r1_data2);

		*status = val2 & AR8327_ATU_STATUS;
		if (!*status)
			break;

		i = 0;
		t = AR8327_ATU_PORT0;
		while (!(val1 & t) && ++i < AR8327_NUM_PORTS)
			t <<= 1;

		a->port = i;
		a->mac[0] = (val0 & AR8327_ATU_ADDR0) >> AR8327_ATU_ADDR0_S;
		a->mac[1] = (val0 & AR8327_ATU_ADDR1) >> AR8327_ATU_ADDR1_S;
		a->mac[2] = (val0 & AR8327_ATU_ADDR2) >> AR8327_ATU_ADDR2_S;
		a->mac[3] = (val0 & AR8327_ATU_ADDR3) >> AR8327_ATU_ADDR3_S;
		a->mac[4] = (val1 & AR8327_ATU_ADDR4) >> AR8327_ATU_ADDR4_S;
		a->mac[5] = (val1 & AR8327_ATU_ADDR5) >> AR8327_ATU_ADDR5_S;
		break;
	}
}
#endif

static int
ar8327_sw_hw_apply(struct switch_dev *dev)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	const struct ar8327_data *data = priv->chip_data;
	int ret, i;

	ret = ar8xxx_sw_hw_apply(dev);
	if (ret)
		return ret;

	for (i=0; i < AR8XXX_NUM_PHYS; i++) {
		if (data->eee[i])
			ar8xxx_reg_clear(priv, AR8327_REG_EEE_CTRL,
			       AR8327_EEE_CTRL_DISABLE_PHY(i));
		else
			ar8xxx_reg_set(priv, AR8327_REG_EEE_CTRL,
			       AR8327_EEE_CTRL_DISABLE_PHY(i));
	}

	return 0;
}

int
ar8327_sw_get_port_igmp_snooping(struct switch_dev *dev,
				 const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	int port = val->port_vlan;

	if (port >= dev->ports)
		return -EINVAL;

	mutex_lock(&priv->reg_mutex);
	val->value.i = ar8327_get_port_igmp(priv, port);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

int
ar8327_sw_set_port_igmp_snooping(struct switch_dev *dev,
				 const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	int port = val->port_vlan;

	if (port >= dev->ports)
		return -EINVAL;

	mutex_lock(&priv->reg_mutex);
	ar8327_set_port_igmp(priv, port, val->value.i);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

int
ar8327_sw_get_igmp_snooping(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	int port;

	for (port = 0; port < dev->ports; port++) {
		val->port_vlan = port;
		if (ar8327_sw_get_port_igmp_snooping(dev, attr, val) ||
		    !val->value.i)
			break;
	}

	return 0;
}

int
ar8327_sw_set_igmp_snooping(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	int port;

	for (port = 0; port < dev->ports; port++) {
		val->port_vlan = port;
		if (ar8327_sw_set_port_igmp_snooping(dev, attr, val))
			break;
	}

	return 0;
}

int
ar8327_sw_get_igmp_v3(struct switch_dev *dev,
		      const struct switch_attr *attr,
		      struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);
	u32 val_reg;

	mutex_lock(&priv->reg_mutex);
	val_reg = ar8xxx_read(priv, AR8327_REG_FRAME_ACK_CTRL1);
	val->value.i = ((val_reg & AR8327_FRAME_ACK_CTRL_IGMP_V3_EN) != 0);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

int
ar8327_sw_set_igmp_v3(struct switch_dev *dev,
		      const struct switch_attr *attr,
		      struct switch_val *val)
{
	struct ar8xxx_priv *priv = swdev_to_ar8xxx(dev);

	mutex_lock(&priv->reg_mutex);
	if (val->value.i)
		ar8xxx_reg_set(priv, AR8327_REG_FRAME_ACK_CTRL1,
			       AR8327_FRAME_ACK_CTRL_IGMP_V3_EN);
	else
		ar8xxx_reg_clear(priv, AR8327_REG_FRAME_ACK_CTRL1,
				 AR8327_FRAME_ACK_CTRL_IGMP_V3_EN);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static const struct switch_attr ar8327_sw_attr_globals[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_vlan",
		.description = "Enable VLAN mode",
		.set = ar8xxx_sw_set_vlan,
		.get = ar8xxx_sw_get_vlan,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "iface_mode",
		.description = "Enable Interface mode",
		.set = ar8xxx_sw_set_iface_mode,
		.get = ar8xxx_sw_get_iface_mode,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_STRING,
		.name = "ag71xx_ifname",
		.description = "Network iface name for ag71xx driver",
		.set = ar8xxx_sw_set_ag71xx_ifname,
		.get = ar8xxx_sw_get_ag71xx_ifname,
	},
	{
		.type = SWITCH_TYPE_NOVAL,
		.name = "reset_mibs",
		.description = "Reset all MIB counters",
		.set = ar8xxx_sw_set_reset_mibs,
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_mirror_rx",
		.description = "Enable mirroring of RX packets",
		.set = ar8xxx_sw_set_mirror_rx_enable,
		.get = ar8xxx_sw_get_mirror_rx_enable,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_mirror_tx",
		.description = "Enable mirroring of TX packets",
		.set = ar8xxx_sw_set_mirror_tx_enable,
		.get = ar8xxx_sw_get_mirror_tx_enable,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "mirror_monitor_port",
		.description = "Mirror monitor port",
		.set = ar8xxx_sw_set_mirror_monitor_port,
		.get = ar8xxx_sw_get_mirror_monitor_port,
		.max = AR8327_NUM_PORTS - 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "mirror_source_port",
		.description = "Mirror source port",
		.set = ar8xxx_sw_set_mirror_source_port,
		.get = ar8xxx_sw_get_mirror_source_port,
		.max = AR8327_NUM_PORTS - 1
 	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "arl_age_time",
		.description = "ARL age time (secs)",
		.set = ar8xxx_sw_set_arl_age_time,
		.get = ar8xxx_sw_get_arl_age_time,
	},
	{
		.type = SWITCH_TYPE_STRING,
		.name = "arl_table",
		.description = "Get ARL table",
		.set = NULL,
		.get = ar8xxx_sw_get_arl_table,
	},
	{
		.type = SWITCH_TYPE_NOVAL,
		.name = "flush_arl_table",
		.description = "Flush ARL table",
		.set = ar8xxx_sw_set_flush_arl_table,
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "igmp_snooping",
		.description = "Enable IGMP Snooping",
		.set = ar8327_sw_set_igmp_snooping,
		.get = ar8327_sw_get_igmp_snooping,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "igmp_v3",
		.description = "Enable IGMPv3 support",
		.set = ar8327_sw_set_igmp_v3,
		.get = ar8327_sw_get_igmp_v3,
		.max = 1
	},
};

static const struct switch_attr ar8327_sw_attr_port[] = {
	{
		.type = SWITCH_TYPE_NOVAL,
		.name = "reset_mib",
		.description = "Reset single port MIB counters",
		.set = ar8xxx_sw_set_port_reset_mib,
	},
	{
		.type = SWITCH_TYPE_STRING,
		.name = "mib",
		.description = "Get port's MIB counters",
		.set = NULL,
		.get = ar8xxx_sw_get_port_mib,
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_eee",
		.description = "Enable EEE PHY sleep mode",
		.set = ar8327_sw_set_eee,
		.get = ar8327_sw_get_eee,
		.max = 1,
	},
	{
		.type = SWITCH_TYPE_NOVAL,
		.name = "flush_arl_table",
		.description = "Flush port's ARL table entries",
		.set = ar8xxx_sw_set_flush_port_arl_table,
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "igmp_snooping",
		.description = "Enable port's IGMP Snooping",
		.set = ar8327_sw_set_port_igmp_snooping,
		.get = ar8327_sw_get_port_igmp_snooping,
		.max = 1
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "advertise",
		.description = "Auto-negotiation advertised port speeds",
		.set = ar8xxx_sw_set_advertise_speeds,
		.get = ar8xxx_sw_get_advertise_speeds,
	},
};

static const struct switch_dev_ops ar8327_sw_ops = {
	.attr_global = {
		.attr = ar8327_sw_attr_globals,
		.n_attr = ARRAY_SIZE(ar8327_sw_attr_globals),
	},
	.attr_port = {
		.attr = ar8327_sw_attr_port,
		.n_attr = ARRAY_SIZE(ar8327_sw_attr_port),
	},
	.attr_vlan = {
		.attr = ar8xxx_sw_attr_vlan,
		.n_attr = ARRAY_SIZE(ar8xxx_sw_attr_vlan),
	},
	.get_port_pvid = ar8xxx_sw_get_pvid,
	.set_port_pvid = ar8xxx_sw_set_pvid,
	.get_vlan_ports = ar8327_sw_get_ports,
	.set_vlan_ports = ar8327_sw_set_ports,
	.apply_config = ar8327_sw_hw_apply,
	.reset_switch = ar8xxx_sw_reset_switch,
	.get_port_link = ar8xxx_sw_get_port_link,
	.set_port_link = ar8327_set_port_link,
};

const struct ar8xxx_chip ar8327_chip = {
	.caps = AR8XXX_CAP_GIGE | AR8XXX_CAP_MIB_COUNTERS,
	.config_at_probe = true,
	.mii_lo_first = true,

	.name = "Atheros AR8327",
	.ports = AR8327_NUM_PORTS,
	.vlans = AR8X16_MAX_VLANS,
	.swops = &ar8327_sw_ops,

	.reg_port_stats_start = 0x1000,
	.reg_port_stats_length = 0x100,
	.reg_arl_ctrl = AR8327_REG_ARL_CTRL,

	.hw_init = ar8327_hw_init,
	.cleanup = ar8327_cleanup,
	.init_globals = ar8327_init_globals,
	.init_port = ar8327_init_port,
	.setup_port = ar8327_setup_port,
	.read_port_status = ar8327_read_port_status,
	.read_port_eee_status = ar8327_read_port_eee_status,
	.atu_flush = ar8327_atu_flush,
	.atu_flush_port = ar8327_atu_flush_port,
	.vtu_flush = ar8327_vtu_flush,
	.vtu_load_vlan = ar8327_vtu_load_vlan,
	.phy_fixup = ar8327_phy_fixup,
	.set_mirror_regs = ar8327_set_mirror_regs,
#if 0
	.get_arl_entry = ar8327_get_arl_entry,
#endif
	.sw_hw_apply = ar8327_sw_hw_apply,

	.num_mibs = ARRAY_SIZE(ar8236_mibs),
	.mib_decs = ar8236_mibs,
	.mib_func = AR8327_REG_MIB_FUNC
};

const struct ar8xxx_chip ar8337_chip = {
	.caps = AR8XXX_CAP_GIGE | AR8XXX_CAP_MIB_COUNTERS,
	.config_at_probe = true,
	.mii_lo_first = true,

	.name = "Atheros AR8337",
	.ports = AR8327_NUM_PORTS,
	.vlans = AR8X16_MAX_VLANS,
	.swops = &ar8327_sw_ops,

	.reg_port_stats_start = 0x1000,
	.reg_port_stats_length = 0x100,
	.reg_arl_ctrl = AR8327_REG_ARL_CTRL,

	.hw_init = ar8327_hw_init,
	.cleanup = ar8327_cleanup,
	.init_globals = ar8327_init_globals,
	.init_port = ar8327_init_port,
	.setup_port = ar8327_setup_port,
	.read_port_status = ar8327_read_port_status,
	.read_port_eee_status = ar8327_read_port_eee_status,
	.atu_flush = ar8327_atu_flush,
	.atu_flush_port = ar8327_atu_flush_port,
	.vtu_flush = ar8327_vtu_flush,
	.vtu_load_vlan = ar8327_vtu_load_vlan,
	.phy_fixup = ar8327_phy_fixup,
	.set_mirror_regs = ar8327_set_mirror_regs,
#if 0
	.get_arl_entry = ar8327_get_arl_entry,
#endif
	.sw_hw_apply = ar8327_sw_hw_apply,

	.num_mibs = ARRAY_SIZE(ar8236_mibs),
	.mib_decs = ar8236_mibs,
	.mib_func = AR8327_REG_MIB_FUNC
};
