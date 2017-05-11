#ifndef __AG71XX_SLAVES_H
#define __AG71XX_SLAVES_H

struct net_device *get_slave_dev_by_port_num(int port_num);
struct ag71xx_slave *get_slave_ags_by_port_num(int port_num);
void create_slave_devices(struct ag71xx *ag);
void destroy_slave_devices(struct ag71xx *ag);

#endif /* __AG71XX_SLAVES_H */
