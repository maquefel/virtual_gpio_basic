#DESCRIPTION
------------

Virtual gpio with irq_chip module for using together with ivshmem qemu



#REQUIREMENTS
------------

linux kernel 4.6 (or just headers)  
qemu 2.5.1.1 

#COMPILATION
------------

KDIR={$PATH_TO_KERNEL_SOURCE_DIRECTORY} QEMU_DIR={$PATH_TO_QEMU_SOURCES_DIRECTORY} make all

optional ARCH={$ARCH}
optional CROSS_COMPILE={$CROSSCOMPILER_PREFIX}


#USAGE
------------

##WITHOUT INTERRUPTS 

```bash
$ qemu-system-{$ARCH} (...) -device ivshmem,shm=ivshmem,size=1

guest machine:
# insmod virtual_gpio_basic.ko

# ls /sys/class/gpio/
export       gpiochip248  unexport

# cat /sys/class/gpio/gpiochip248/label
0000:00:0d.0

# cat /sys/class/gpio/gpiochip248/base
248

# cat /sys/class/gpio/gpiochip248/ngpio
32

# echo 248 > /sys/class/gpio/export
# echo high > /sys/class/gpio/gpio248/direction

host machine:
$ xxd -b -l 2 -c 2 /dev/shm/ivshmem
0000000: 00000001 00000001  ..

vg_get_set:

$ ./vg_get_set -r 0
main:222:: read 0=1

$ ./vg_get_set -i 1
main:239:: write 1=1

guest machine:

# cat gpio1/value
1
```

##WITH INTERRUPTS

```bash
host machine:
$ ivshmem-server -F -v -l 1M -n 1
$ qemu-system-{$ARCH} (...) -chardev socket,path=/tmp/ivshmem_socket,id=ivshmemid -device ivshmem,chardev=ivshmemid,size=1,msi=off

guest machine:
# insmod virtual_gpio_basic.ko

# echo 248 > /sys/class/gpio/export
# echo 249 > /sys/class/gpio/export
# echo 250 > /sys/class/gpio/export
# echo rising > /sys/class/gpio/gpio248/edge
# echo rising > /sys/class/gpio/gpio249/edge
# echo rising > /sys/class/gpio/gpio250/edge

# ./vg_guest_client 248
gpio_chip:
        base: 248
        ngpio: 32
Added gpio 248 to watchlist.
Added gpio 249 to watchlist.
Added gpio 250 to watchlist.
Entering loop with 3 gpios.

host machine:
$ ./vg_get_set -p 6 -i 0 # get -p (peer id) from ivshmem-client
main:346:: write 0=1
main:349:: interrupt was enabled for 0
main:353:: interrupt was raised for 0
main:357:: rising edge interrupt was enabled for 0 and new value is high 1

guest:

VGPIO: interrupt (status = 0x0001, pending = 1)
VGPIO: interrupt (status = 0x0001)
VGPIO: interrupt (status = 0x0001)
gpio number=248 interrupt caught
```

#COPYRIGHT
------------

GPL
Nikita Shubin <maquefel@gmail.com>
