
#include <linux/module.h>
#include <linux/init.h>

static int __init hello_init(void)
{
	printk("I'm a example for kernel module.\n");
	
	return 0;
}


static void __exit hello_exit(void)
{
	printk("[example] EXIT... \n");
	
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leon Wang");
