#make по ':all' собирает сразу модули для x86 и mips
ifndef ARCH
  export ARCH = x86
  export KERNEL_DIR = /lib/modules/$(shell uname -r)/build
endif
ifeq ($(ARCH),mips)
 include $(PWD)/lede-mips.mk
endif
ifeq ($(ARCH),arm)
 include $(PWD)/arm.mk
endif

# standard flags for module builds
EXTRA_CFLAGS += -DLINUX -D__KERNEL__ -DMODULE -O2 -pipe -Wall

TARGET=test_m.o
obj-m:=$(TARGET)
test_m-objs:=test_main.o


TARGETS := $(obj-m:.o=.ko)
ccflags-y += -Wall

all:
	make x86
#	make arm
#	make mips
#	make e2k

arch:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$$PWD modules

x86:
	$(MAKE) arch ARCH=x86

mips:
	$(MAKE) arch ARCH=mips

arm:
	$(MAKE) arch ARCH=arm

e2k:
	$(MAKE) arch ARCH=e2k

clean:		
		rm -f .*.cmd *.mod.c *.ko *.o *~ core $(TARGETS)
		rm -fr .tmp_versions
