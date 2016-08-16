#ifndef _VIRTUAL_GPIO_H_
#define _VIRTUAL_GPIO_H_

#define VIRTUAL_GPIO_DEV_NAME "ivshmem_gpio"
#define VIRTUAL_GPIO_NR_GPIOS 32

#define VIRTUAL_GPIO_DATA       0x00
#define VIRTUAL_GPIO_OUT_EN     0x04
#define VIRTUAL_GPIO_INT_EN     0x08
#define VIRTUAL_GPIO_INT_ST     0x0c
#define VIRTUAL_GPIO_INT_EOI    0x10
#define VIRTUAL_GPIO_RISING     0x14
#define VIRTUAL_GPIO_FALLING    0x18

#endif
