#cross compile для arm
#root
WRTP = /home/prog/android/samsung/j3
#где лежат бинарники компилятора(gcc, ldd)
#PATH := $(PATH):$(STAGING_DIR)/bin
#указываем архитекруту
export ARCH = arm
export LD_LIBRARY_PATH=$(WRTP)/arm-eabi-4.8/arch-arm/usr/lib
export CROSS_COMPILE=$(WRTP)/arm-eabi-4.8/bin/arm-eabi-
#дальше все как обычно для модуля
#путь к исходнику ядра
KERNEL_DIR = $(WRTP)/kernel
