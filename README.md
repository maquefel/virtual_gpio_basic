# DESCRIPTION

PCI virtual gpio with interrupts support for using together with ivshmem qemu

See original articles for older versions:

- http://maquefel.me/en/qemu-ivshmem-introducing-interrupt-capable-virtual-gpio-concept/
- http://maquefel.me/en/using-gpio-generic-and-irq_chip_generic-subsystems-for-gpio-driver/

# REQUIREMENTS

- linux kernel
- qemu

This version is done for usage with 5.12 linux kernel and 6.0 qemu

# COMPILATION

KDIR={$PATH_TO_KERNEL_SOURCE_DIRECTORY} QEMU_DIR={$PATH_TO_QEMU_SOURCES_DIRECTORY} make all

optional ARCH={$ARCH}
optional CROSS_COMPILE={$CROSSCOMPILER_PREFIX}

# QUICKSTART

Use https://github.com/maquefel/kernel-bisect-template/tree/virtual_gpio project for quickstart.

# USAGE

I have dropped non-irq based version and switched to MSI vector usage. The modules simply won't load with error (it can be fixed easily if you really require that).

See https://github.com/qemu/qemu/blob/master/docs/specs/ivshmem-spec.txt for additional info.

## WITH INTERRUPTS

```
host machine:
$ ./qemu/build/contrib/ivshmem-server/ivshmem-server -F -v -l 1M -n 1
$ qemu-system-x86_64 -cpu host -kernel build-linux/arch/x86/boot/bzImage -initrd initramfs.cpio.xz -nographic -append "nokaslr console=ttyS0 root=/dev/ram" -chardev socket,path=/tmp/ivshmem_socket,id=ivshmemid -device ivshmem-doorbell,chardev=ivshmemid,vectors=1 -enable-kvm

guest # lsgpio
...
GPIO chip: gpiochip4, "0000:00:04.0", 32 GPIO lines
        line  0: unnamed unused [input]
        line  1: unnamed unused [input]
        line  2: unnamed unused [input]
        line  3: unnamed unused [input]
...

guest # /usr/bin/gpio-event-mon -n gpiochip4 -o 0

host $ ./vg_get_set -p 6 -i 0 # get -p (peer id) from ivshmem-client
main:346:: write 0=1
main:349:: interrupt was enabled for 0
main:353:: interrupt was raised for 0
main:357:: rising edge interrupt was enabled for 0 and new value is high 1

guest:
No flags specified, listening on both rising and falling edges
Monitoring line 0 on gpiochip4
Initial line value: 0

GPIO EVENT at 33533945213 on line 0 (1|1) rising edge
GPIO EVENT at 39910880760 on line 0 (2|2) falling edge
GPIO EVENT at 44507121043 on line 0 (3|3) rising edge
```

# COPYRIGHT

GPL
Nikita Shubin <nikita.shubin@maquefel.me>
