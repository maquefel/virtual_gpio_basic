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

#include <stdbool.h>

#include <sys/stat.h>
#include <sys/mman.h>

#include "ivshmem-client.h"

#include "virtual_gpio_basic.h"

#define reg_size int32_t

#ifndef BITS_PER_BYTE
#define BITS_PER_BYTE 8
#endif

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

#ifndef BITS_TO_BYTES
#define BITS_TO_BYTES(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE)
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

#define IVSHMEM_CLIENT_DEFAULT_VERBOSE        0
#define IVSHMEM_CLIENT_DEFAULT_UNIX_SOCK_PATH "/tmp/ivshmem_socket"

typedef struct IvshmemClientArgs {
    bool verbose;
    const char *unix_sock_path;
} IvshmemClientArgs;

static struct option long_options[] =
{
{"version",     no_argument,        0,                  'v'},
{"device",      required_argument,  0,                  'd'},
{"write-low",   required_argument,  0,                  'l'},
{"write-high",  required_argument,  0,                  'i'},
{"peer",        required_argument,  0,                  'p'},
{"ngpio",       required_argument,  0,                  'n'},
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
                "-p, --peer=ID         notify peer_id of interrupt\n" \
                "-n, --ngpio=CNT       specify gpio number\n" \
                "-h, --help            prints this message\n");
    }
}

/* listen on stdin (command line), on unix socket (notifications of new
 * and dead peers), and on eventfd (IRQ request) */
static int
ivshmem_client_get_peer(IvshmemClient *client, int peer_id)
{
    fd_set fds;
    int ret, maxfd = 0;


    do {
        FD_ZERO(&fds);
        maxfd = 0;
        ivshmem_client_get_fds(client, &fds, &maxfd);

        ret = select(maxfd, &fds, NULL, NULL, NULL);

        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            fprintf(stderr, "select error: %s\n", strerror(errno));
            break;
        }
        if (ret == 0) {
            continue;
        }

        if (FD_ISSET(client->sock_fd, &fds) && ivshmem_client_handle_fds(client, &fds, maxfd) < 0) {
            fprintf(stderr, "ivshmem_client_handle_stdin_command() failed\n");
            break;
        }

    } while(ivshmem_client_search_peer(client, peer_id) == NULL);

    return ret;
}

/* callback when we receive a notification (just display it) */
static void ivshmem_client_notification_cb(const IvshmemClient *client,
                               const IvshmemClientPeer *peer,
                               unsigned vect, void *arg)
{
    (void)client;
    (void)arg;

    printf("receive notification from peer_id=%d vector=%u\n", peer->id, vect);
}

static unsigned int ngpio = VIRTUAL_GPIO_NR_GPIOS;

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
    int ret = 0;

    int fd;
    void *map = NULL;
    ssize_t filesize = 0;
    struct ivshmem_data ivd;
    unsigned nr;
    unsigned val;    
    const char *filename = NULL;

    int peer_id, vector = 0;

    ivd.filename = "/dev/shm/ivshmem"; /* default '/dev' node */
    ivd.filesize = 0x100000;    /* default mmio region size */
    ivd.ivshmem_op = NE_IVSHMEM_READ;   /* default op */

    IvshmemClient client;
    IvshmemClientArgs args = {
        .verbose = IVSHMEM_CLIENT_DEFAULT_VERBOSE,
        .unix_sock_path = IVSHMEM_CLIENT_DEFAULT_UNIX_SOCK_PATH,
    };

    int c = -1;
    int option_index = 0;

    while((c = getopt_long(argc, argv, "vhd:l:i:r:p:n:", long_options, &option_index)) != -1) {
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
        case 'p':
            peer_id = strtol(optarg, 0, 10);
            break;
        case 'h':
            PrintUsage(argc, argv);
            exit(EXIT_SUCCESS);
            break;
        case 'n':
            ngpio = strtol(optarg, 0, 10);
        default:
            break;
        }
    }

    if(nr > ngpio)
    {
        printf("Ping number is out of range. Number of gpios: %d\n", VIRTUAL_GPIO_NR_GPIOS);
        return EINVAL;
    }

    unsigned int nbytes = BITS_TO_BYTES(ngpio);

    filename = ivd.filename;
    filesize = ivd.filesize;

    if (ivshmem_client_init(&client, args.unix_sock_path,
                            ivshmem_client_notification_cb,
                            NULL,
                            args.verbose) < 0) {
        fprintf(stderr, "cannot init client\n");
        return 1;
    }

    if (ivshmem_client_connect(&client) < 0) {
        fprintf(stderr, "ivshmem_client: cannot connect to server.d\n");
        exit(EXIT_FAILURE);
    }

    IvshmemClientPeer *peer = 0;

    ivshmem_client_get_peer(&client, peer_id);
    peer = ivshmem_client_search_peer(&client, peer_id);

#ifdef DEBUG
    ivshmem_client_dump(&client);
#endif

    if (peer == NULL) {
        printf("cannot find peer_id = %d\n", peer_id);
        exit(EXIT_FAILURE);
    }

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

    reg_size *data, *output, *i_en, *i_st, *r_edge, *f_edge, *eoi;

    reg_size mask = (1 << nr);

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
        i_en = map + VIRTUAL_GPIO_INT_EN;
        i_st = map + VIRTUAL_GPIO_INT_ST;
        r_edge = map + VIRTUAL_GPIO_RISING;
        f_edge = map + VIRTUAL_GPIO_FALLING;
        eoi = map + VIRTUAL_GPIO_INT_EOI;

        if(!!(*output & (1 << nr)) == 1) { // rework shift to mask
            prinfo("pin configurated as output not writing %d=%d\n", nr, val);
            break;
        }

        if (val)
            *data |= (1 << nr);
        else
            *data &= ~(1 << nr);

        prinfo("write %d=%d\n", nr, val);

        if(!!(*i_en & (1 << nr)) == 1) {
            prinfo("interrupt was enabled for %d\n", nr);

            /* check if interrupt already raised */            
            if(!!(*i_st & (1 << nr)) == 0) {
                prinfo("interrupt was raised for %d\n", nr);

                if(val && (!!(*r_edge & (1 << nr)) == 1))
                {
                    prinfo("rising edge interrupt was enabled for %d and new value is high %d\n", nr, val);

                    /* set interrupt bit and raise interrupt */
                    *i_st |= (1 << nr);
                    ivshmem_client_notify(&client, peer, vector);
                    /* wait for eoi => move to sem */
                    /* eoi fast check and clear interrupt */
                    /* add counter check */
                    do {} while(!!(*eoi & (1 << nr)) != 1);
                    *i_st &= ~(1 << nr);

                    /* eoi should be always zero */
                    *eoi &= ~(1 << nr);
                }

                if(!val && (!!(*f_edge & (1 << nr)) == 1))
                {
                    prinfo("falling edge interrupt was enabled for %d and new value is low %d\n", nr, val);

                    /* set interrupt bit and raise interrupt */
                    *i_st |= (1 << nr);
                    ivshmem_client_notify(&client, peer, vector);
                    /* wait for eoi => move to sem */
                    /* eoi fast check and clear interrupt */
                    /* add counter check */
                    do {} while(!!(*eoi & (1 << nr)) != 1);
                    *i_st &= ~(1 << nr);

                    /* eoi should be always zero */
                    *eoi &= ~(1 << nr);
                }
            }            
        }
        break;
    default:
        prinfo("no read/write operations performed\n");
    }

    if ((munmap(map, filesize)) < 0)
        prerr("WARNING: Failed to munmap \"%s\"\n", filename);

peer_failure:
fd_failure:
    close(fd);
    ivshmem_client_close(&client);

    return ret;
}
