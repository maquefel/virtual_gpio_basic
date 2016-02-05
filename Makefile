#
# file : Makefile
# desc : Build linux device driver and userspace sources for
#         test with the QEMU ivshmem PCI device
#

usr = vg_get_set vg_guest_client
OBJS := vg_get_set.o ivshmem-client.o
krn = virtual_gpio_basic

ifneq ($(KERNELRELEASE),)

obj-m := $(krn).o

else

CFLAGS = `pkg-config --cflags glib-2.0` 
LDLIBS = `pkg-config --libs glib-2.0`

KDIR ?= /lib/modules/$$(uname -r)/build
QEMU_DIR ?= ../qemu-2.5.0/
CC = $(CROSS_COMPILE)gcc

vg_get_set: $(OBJS)
	gcc $(OBJS) -o vg_get_set $(LDLIBS)

-include $(OBJS:.o=.d)

%.o: %.c
	gcc -I. -I$(QEMU_DIR) -I$(QEMU_DIR)/include -c $(CFLAGS) $*.c -o $*.o
	gcc -I. -I$(QEMU_DIR) -I$(QEMU_DIR)/include -MM $(CFLAGS) $*.c > $*.d

vg_guest_client:
	$(CC) -Wall -O2 vg_guest_client.c -o vg_guest_client

module:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
	rm -f $(usr)

all: module vg_get_set vg_guest_client

debug: CFLAGS += -DDEBUG -g
debug: vg_get_set

.PHONY : clean
endif
