#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <net/ip.h>
#include <net/icmp.h>
#include <net/udp.h>
#include <net/route.h>
#include <linux/pkt_sched.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_tcpudp.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/if_arp.h>
#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/err.h>
#include <net/protocol.h>
#include "ag71xx.h"
#include "nnn.h"

#define DEBUG 1
#include "debug.h"

/* !!! Перед использованием обнови весь код макросов, структур и функций из ag71xx_*.c !!!
	 !!! на новых версиях он может не совпадать !!!*/


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sergey Sergeev <sergey.sergeev@yapic.net>");
MODULE_DESCRIPTION("kernel test");

//u32 (*ar7240sw_reg_read)(struct mii_bus *mii, u32 reg_addr);
//u32 (*ar7240sw_reg_write)(struct mii_bus *mii, u32 reg_addr, u32 reg_val);
u32 ar7240sw_reg_read(struct mii_bus *mii, u32 reg_addr);
void ar7240sw_reg_write(struct mii_bus *mii, u32 reg_addr, u32 reg_val);

//*********************************************************
//выполняется при загрузке модуля
static int __init test_m_module_init(void){
  struct net_device *oif_dev;
	u32 vlan1; u32 vlan2; u32 ctrl;
	u32 cpu_port;
  //int j;

/*
  //это для случая когда ag71xx влинкован в ядро.
  //для загружаемого модуля этот способ почемуто не работает!
  ar7240sw_reg_read = (void*)kallsyms_lookup_name("ar7240sw_reg_read");
  ar7240sw_reg_write = (void*)kallsyms_lookup_name("ar7240sw_reg_write");
  if(!ar7240sw_reg_read || !ar7240sw_reg_write){
    printk(KERN_ERR "Can't find ar7240sw_reg_r/w ptrs!\n");
    return -EINVAL;
  } */

  oif_dev = dev_get_by_name(&init_net, "eth2");
  if(likely(oif_dev)){
    struct ag71xx *ag;
		struct mii_bus *mii;
	  struct ar7240sw *as;
 		struct switch_dev *swdev;
    PRINTD("oif %s, %d is found!\n", oif_dev->name, oif_dev->ifindex);
		/* это магия обхода инкапсуляции */

		ag = netdev_priv(oif_dev);
		as = ag->phy_priv;
		if(!as)
			goto end;
		swdev = &as->swdev;

		// < ----
	  PRINTD("info->driver = %s\n", ag->pdev->dev.driver->name);
  	PRINTD("info->version = %s\n", AG71XX_DRV_VERSION);
	  PRINTD("info->bus_info = %s\n", dev_name(&ag->pdev->dev));
		mii = ag->mii_bus;
	  PRINTD("ag->mii_bus = 0x%p\n", mii);
	  PRINTD("swdev->name = %s\n", swdev->name);
	  PRINTD("swdev->ports = %d\n", swdev->ports);
	  PRINTD("as->vlan = %d\n", as->vlan);
	  PRINTD("as->vlan_tagged = %d\n", as->vlan_tagged);
		PRINTD("ar7240sw_reg_read = 0x%p\n", ar7240sw_reg_read);
		//goto end;

		{ //читаем статусы портов
			u32 status;
			int a;
			for(a = 0; a < 5; a++){
				status = ar7240sw_reg_read(mii, AR7240_REG_PORT_STATUS(a));
				vlan1 = ar7240sw_reg_read(mii, AR934X_REG_PORT_VLAN1(a));
				vlan2 = ar7240sw_reg_read(mii, AR934X_REG_PORT_VLAN2(a));
				ctrl = ar7240sw_reg_read(mii, AR7240_REG_PORT_CTRL(a));
				printk(KERN_INFO "port %d status = 0x%x,  vlan1_d = 0x%x, vlan2_d = 0x%x, ctrl = 0x%x\n",
							 a, status, vlan1, vlan2, ctrl	);
				//разрешаем всем портам видеть друг дружку(!всЁ!)
				//ar7240sw_reg_write(mii, AR934X_REG_PORT_VLAN2(a),
					//								 vlan2 | (0x3F << AR934X_PORT_VLAN2_PORT_VID_MEM_S));
				//disable port
				ar7240sw_reg_write(mii, AR7240_REG_PORT_CTRL(a), AR7240_PORT_CTRL_STATE_DISABLED);
			}
		}

		//set cpu port and enable mirroring to it
		cpu_port = 	ar7240sw_reg_read(mii, AR7240_REG_CPU_PORT);
		printk(KERN_INFO "cpu port = 0x%x\n", cpu_port);
		//!ar7240sw_reg_write(mii, AR7240_REG_CPU_PORT, 0x100);

		//enable atheros header
		//ctrl = ar7240sw_reg_read(mii, AR7240_REG_PORT_CTRL(3));
		//ctrl |= AR7240_PORT_CTRL_HEADER;
		//ctrl &= ~AR7240_PORT_CTRL_HEADER;
		//ar7240sw_reg_write(mii, AR7240_REG_PORT_CTRL(3), ctrl);
		//disable learning to ATU
		//ctrl = 0x26004;
 		//ar7240sw_reg_write(mii, AR7240_REG_PORT_CTRL(1), ctrl);
		//Flush all ATU entries
 		//ar7240sw_reg_write(mii, AR7240_REG_ATU, 0x1);
		//Disable port
		//ctrl = ar7240sw_reg_read(mii, AR7240_REG_PORT_CTRL(3));
 		//ar7240sw_reg_write(mii, AR7240_REG_PORT_CTRL(3), AR7240_PORT_CTRL_STATE_DISABLED);

		//for dump vlan table
		/* for (j = 0; j < AR7240_MAX_VLANS; j++) {
			PRINTD("%d: as->vlan_id = 0x%x, as->vlan_table = 0x%x\n", j, as->vlan_id[j], as->vlan_table[j]);
		} */
end:
    dev_put(oif_dev); //освобождаем ссылку на устройство !
  }
  return -ENOMEM;
}//--------------------------------------------------------

//*********************************************************
//выполняется при выгрузке модуля
static void __exit test_m_module_exit (void){
}//--------------------------------------------------------

module_init(test_m_module_init);
module_exit(test_m_module_exit);
