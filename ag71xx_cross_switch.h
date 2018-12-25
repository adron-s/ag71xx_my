#ifndef __AG71XX_CROSS_SWITCH_H
#define __AG71XX_CROSS_SWITCH_H

//символическое обозначении версии свитча AR8327
#define		EXT_SW_VERSION_AR8327 0x8327

struct mii_bus *ar8327_get_mii_bus_from_swdev(struct switch_dev *);
void ag71xx_cross_sw_set_port_state(struct ag71xx *, unsigned port, int);
int ar71xx_get_sw_num_ports(struct ag71xx *);
int ag71xx_cross_sw_adjust_port_link(struct ag71xx *, u32, struct switch_port_link *, u8);
u16 ag71xx_cross_sw_mdio_read(struct ag71xx *, unsigned, unsigned);
int ag71xx_cross_sw_mdio_write(struct ag71xx *, unsigned, unsigned, u16);
int cross_sw_set_port_link(struct ag71xx *, struct switch_dev *, int, struct switch_port_link *, u8);


#endif /* _AG71XX_CROSS_SWITCH_H */
