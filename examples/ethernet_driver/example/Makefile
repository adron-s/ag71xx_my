# Kernel module example
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

PKG_NAME:=example
PKG_RELEASE:=1


define KernelPackage/example
	SUBMENU:=Other modules
	DEPENDS:=					#@TARGET_octeon
	TITLE:=Support Module for example
	FILES:=$(PKG_BUILD_DIR)/example.$(LINUX_KMOD_SUFFIX)
	AUTOLOAD:=$(call AutoLoad,81,example)
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) -R ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)" \
		NOSTDINC_FLAGS="$(NOSTDINC_FLAGS)" \
		modules
endef

$(eval $(call KernelPackage,example))

