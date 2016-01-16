#
# file : Makefile
# desc : Build linux device driver and userspace sources for
#         test with the QEMU ivshmem PCI device
#

usr = vg_get_set
krn = virtual_gpio_basic

ifneq ($(KERNELRELEASE),)

obj-m := $(krn).o

else

KDIR ?= /lib/modules/$$(uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD modules
	$(CC) -Wall -O2 $(usr).c -o $(usr)

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
	rm -f $(usr)

.PHONY : clean
endif
