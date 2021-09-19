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

KDIR ?= ${CURDIR}/../linux
QEMU_DIR ?= ${CURDIR}/../qemu
SYSROOT ?= ${CURDIR}/../initramfs
CC = $(CROSS_COMPILE)gcc

LINUXCONFIG     += DEBUG=1

vg_get_set: $(OBJS)
	gcc $(OBJS) -o vg_get_set $(LDLIBS)

-include $(OBJS:.o=.d)

%.o: %.c
	gcc -I. -I$(QEMU_DIR) -I$(QEMU_DIR)/include -I$(QEMU_DIR)/build -c $(CFLAGS) $*.c -o $*.o
	gcc -I. -I$(QEMU_DIR) -I$(QEMU_DIR)/include -I$(QEMU_DIR)/build -MM $(CFLAGS) $*.c > $*.d

modules:
	$(MAKE) $(LINUXCONFIG) -C $(KDIR) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD INSTALL_MOD_PATH=${SYSROOT} modules_install

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
	rm -f $(usr)

all: modules vg_get_set vg_guest_client

debug: CFLAGS += -DDEBUG -g
debug: vg_get_set

.PHONY : clean
endif
