#
# Makefile for the Atheros AR71xx built-in ethernet macs
#

include $(PWD)/lede-mips.mk

CONFIG_AG71XX = m
CONFIG_AG71XX_AR9344_SUPPORT = y

ag71xx-y	+= ag71xx_main.o
ag71xx-y	+= ag71xx_ethtool.o
ag71xx-y	+= ag71xx_phy.o
ag71xx-y	+= ag71xx_mdio.o
ag71xx-y	+= ag71xx_ar7240.o

ag71xx-$(CONFIG_AG71XX_DEBUG_FS)	+= ag71xx_debugfs.o
ag71xx-$(CONFIG_AG71XX_AR8216_SUPPORT)	+= ag71xx_ar8216.o

obj-$(CONFIG_AG71XX)	+= ag71xx.o

# standard flags for module builds
EXTRA_CFLAGS += -DLINUX -D__KERNEL__ -DMODULE -O2 -pipe -Wall

ifdef CONFIG_AG71XX_AR9344_SUPPORT
  EXTRA_CFLAGS += -DCONFIG_AG71XX_AR9344_SUPPORT
endif

all:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$$PWD modules

clean:		
		rm -f .*.cmd *.mod.c *.ko *.o *~ core $(TARGETS)
		rm -fr .tmp_versions
