#
#  Makefile
#
#  Created on: March 11, 2023
#      Author: Dylan Robinson
#

ifneq ($(KERNELRELEASE),)

	snd-usb-motu-objs := motu-usb-audio.o

	# ccflags-y := -I./include
	# ccflags-y += -DMOTU

	obj-m += snd-usb-motu.o

else

KDIR ?= /lib/modules/`uname -r`/build
TARGET := snd-usb-motu.ko

default:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean

install:
	install -D $(TARGET) $(DESTDIR)/lib/modules/$(TARGET)

endif
