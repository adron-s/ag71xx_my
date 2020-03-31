#cross compile для mips
#путь где лежит buildroot openwrt
WRTP = /home/prog/openwrt/2020-openwrt/openwrt-18.06.8
#эта переменная нужна компиляроту. указывает на диру с toolchain.
export STAGING_DIR = $(WRTP)/staging_dir/toolchain-mips_24kc_gcc-7.3.0_musl
#где лежат бинарники компилятора(gcc, ldd)
PATH := $(PATH):$(STAGING_DIR)/bin
#указываем архитекруту
export ARCH = mips
#указываем какой префикс юзать перед сажем gcc или ldd(то есть какое имя у файла бинарника компилятора или линкера)
export CROSS_COMPILE = mips-openwrt-linux-
#дальше все как обычно для модуля
#путь к исходнику ядра
KERNEL_DIR = $(WRTP)/build_dir/target-mips_24kc_musl/linux-ar71xx_mikrotik/linux-4.9.214

#тут должна быть только одна строка команды. иначе будет ошибка. если нужно
#более одной строки то используй объединение команд посредством ; и ( )
#define ManualBuild/Install
#	mkdir -p $(WRTP)/files/tests
#	cat $(TARGETS) > $(WRTP)/files/tests/$(TARGETS)
#endef
