/*
 * file: vg_get_set.c
 * desc: demo gpio manipulation utility
 *
 * Nikita Shubin, GPLv2
 *
 * derived from:
 *
 * file : ne_ivshmem_shm_guest_usr.c
 * desc : a demo program that updates/reads the ivshmem POSIX SHM region
 *
 * Siro Mugabi, Copyright (c) nairobi-embedded.org, GPLv2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>

#include <sys/stat.h>
#include <sys/mman.h>

#include "virtual_gpio_basic.h"

#define reg_size int8_t

#if VIRTUAL_GPIO_NR_GPIOS != 8
    #define reg_size int16_t
    #if VIRTUAL_GPIO_NR_GPIOS != 16
        #define reg_size int32_t
        #if VIRTUAL_GPIO_NR_GPIOS != 32
            #error only 8,16 or 32 gpios number is allowed
        #endif
    #endif
#endif

#define prfmt(fmt) "%s:%d:: " fmt, __func__, __LINE__
#define prinfo(fmt, ...) printf(prfmt(fmt), ##__VA_ARGS__)
#define prerr(fmt, ...) fprintf(stderr, prfmt(fmt), ##__VA_ARGS__)

#ifndef UNUSED
#define UNUSED(x) (void)(x);
#endif

#ifndef EOK
#define EOK 0
#endif

#ifndef IVSHMEM_FILE
#define IVSHMEM_FILE "/dev/shm/ivshmem"
#endif

#define PACKAGE_NAME "vg_get_set"
#define PACKAGE_VERSION "0.0.0"

struct ivshmem_data {
    const char *filename;
    ssize_t filesize;
    enum {
        NE_IVSHMEM_READ,
        NE_IVSHMEM_WRITE,
    } ivshmem_op;
};

static struct option long_options[] =
{
    {"version",     no_argument,        0,                  'v'},
    {"device",      required_argument,  0,                  'd'},
    {"write-low",   required_argument,  0,                  'l'},
    {"write-high",  required_argument,  0,                  'i'},
    {"help",        no_argument,        0,                  'h'},
    {0, 0, 0, 0}
};


/**************************************************************************
    Function: Print Usage

    Description:
        Output the command-line options for this utility.

    Params:
        @argc - Standard argument count
        @argv - Standard argument array

    Returns:
        returns void always

**************************************************************************/
void PrintUsage(int argc, char *argv[]) {
    UNUSED(argv);
    if(argc >=1 ) {
        printf( "-v, --version         prints version and exits\n" \
                "-d, --device=DEVICE   specify device for usage e.g. [default="IVSHMEM_FILE"]\n" \
                "-r, --read=NR         reads corresponding gpio value, numeration starts from 0\n" \
                "-l, --write-low=NR    write low to corresponding gpio\n" \
                "-i, --write-high=NR   write high to corresponding gpio\n" \
                "-h, --help            prints this message\n");
    }
}

/**************************************************************************
    Function: main

    Description:
        The c standard 'main' entry point function.

    Params:
        @argc - count of command line arguments given on command line
        @argv - array of arguments given on command line

    Returns:
        returns integer which is passed back to the parent process
**************************************************************************/
int main(int argc, char **argv)
{
    int fd;
    void *map = NULL;
    ssize_t filesize = 0;
    struct ivshmem_data ivd;
    unsigned nr;
    unsigned val;
    const char *filename = NULL;

    ivd.filename = "/dev/shm/ivshmem"; /* default '/dev' node */
    ivd.filesize = 0x100000;    /* default mmio region size */
    ivd.ivshmem_op = NE_IVSHMEM_READ;   /* default op */

    int c = -1;
    int option_index = 0;

    while((c = getopt_long(argc, argv, "vhd:l:i:r:", long_options, &option_index)) != -1) {
        switch(c){
        case 'v' :
        {
            printf("%s version %s\n", PACKAGE_NAME, PACKAGE_VERSION);
            exit(EXIT_SUCCESS);
            break;
        }
        case 'd' :
        {
            if(access(optarg, F_OK) != EOK)
            {
                perror("Please give me correct --device option.\n");
                exit(EXIT_FAILURE);
            }

            ivd.filename = optarg;
            break;
        }
        case 'l' :
        case 'i' :
            val = 0;
            if(c == 'i')
                val = 1;
            nr = strtol(optarg, 0, 0);
            ivd.ivshmem_op = NE_IVSHMEM_WRITE;
            break;
        case 'r' :
        {
            nr = strtol(optarg, 0, 0);
            ivd.ivshmem_op = NE_IVSHMEM_READ;
            break;
        }
        case 'h':
            PrintUsage(argc, argv);
            exit(EXIT_SUCCESS);
            break;
        default:
            break;
        }
    }

    filename = ivd.filename;
    filesize = ivd.filesize;

#ifdef DEBUG
    {
        printf("\nYou entered:\n\tfilename = \"%s\", filesize = %d, operation = %d, ",
                     filename, (int)filesize, ivd.ivshmem_op);
        if (ivd.ivshmem_op == NE_IVSHMEM_WRITE)
            printf("output_string = \"%s\"\n\n", usrstrng);
        else
            printf("\n\n");
    }
#endif

    if ((fd = open(filename, O_RDWR)) < 0)
    {
        prerr("%s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    if ((map = mmap(0, filesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
            0)) == (caddr_t)-1)
    {
        fprintf(stderr, "%s\n", strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }

    reg_size *data, *output;

    switch(ivd.ivshmem_op)
    {
        case NE_IVSHMEM_READ:
            data = map;
            val = !!(*data & (1 << nr));
            prinfo("read %d=%d\n", nr, val);
            break;

        case NE_IVSHMEM_WRITE:
            data = map;
            output = map + VIRTUAL_GPIO_OUT_EN;

            if(!!(*output & (1 << nr)) == 1) {
                prinfo("pin configurated as output not writing %d=%d\n", nr, val);
                break;
            }

            if (val)
                *data |= (1 << nr);
            else
                *data &= ~(1 << nr);

            prinfo("write %d=%d\n", nr, val);
            break;

    default:
            prinfo("no read/write operations performed\n");
    }

    if ((munmap(map, filesize)) < 0)
        prerr("WARNING: Failed to munmap \"%s\"\n", filename);

    close(fd);

    return 0;
}
