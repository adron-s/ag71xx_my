#
# Makefile for the Atheros AR71xx built-in ethernet macs
#
# Этот файл отличается от того Makefile который используется внутри дерева ядра.
# По большей части мне тут просто нужно чтобы код скомпилировался и компилятор
# вывел ошибки/предупреждения. В общем для отладки всего этого.

include $(PWD)/lede-mips.mk

CONFIG_AG71XX = m
CONFIG_AR8216 = m
CONFIG_AR8327 = m
CONFIG_AG71XX_ATH_HDR_SUPPORT = y

ag71xx-y	+= ag71xx_main.o
ag71xx-y	+= ag71xx_ethtool.o
ag71xx-y	+= ag71xx_phy.o
ag71xx-y	+= ag71xx_mdio.o
ag71xx-y	+= ag71xx_ar7240.o
ag71xx-y	+= ag71xx_cross_switch.o
ag71xx-y	+= ag71xx_slaves.o
ar8216-y	+= phy/ar8216.o
ar8327-y	+= phy/ar8327.o

ag71xx-$(CONFIG_AG71XX_DEBUG_FS)	+= ag71xx_debugfs.o
ag71xx-$(CONFIG_AG71XX_AR8216_SUPPORT)	+= ag71xx_ar8216.o
ag71xx-$(CONFIG_AG71XX_ATH_HDR_SUPPORT)	+= ag71xx_ath_hdr.o

obj-$(CONFIG_AG71XX)	+= ag71xx.o
obj-$(CONFIG_AR8216)	+= ar8216.o
obj-$(CONFIG_AR8327)	+= ar8327.o

ifdef CONFIG_AG71XX_ATH_HDR_SUPPORT
  EXTRA_CFLAGS += -DCONFIG_AG71XX_ATH_HDR_SUPPORT
endif

all:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$$PWD modules

clean:
		rm -f .*.cmd *.mod.c *.ko *.o *~ core $(TARGETS)
		rm -fr .tmp_versions
		rm -f ./phy/.*.cmd *.mod.c ./phy/*.ko ./phy/*.o ./phy/*~ ./phy/core $(TARGETS)
		rm -fr ./phy/.tmp_versions
