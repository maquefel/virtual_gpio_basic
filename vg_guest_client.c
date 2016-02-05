#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <time.h>

#include <fcntl.h>
#include <errno.h>

#include <string.h>

#include <signal.h>

#define VALID_OR_DIE(x, y) if(x == -1) {perror(y); exit(EXIT_FAILURE);}

#define IRQF_TRIGGER_NONE       0x00000000
#define IRQF_TRIGGER_RISING     0x00000001
#define IRQF_TRIGGER_FALLING    0x00000002

static const struct {
    const char *name;
    unsigned long flags;
} trigger_types[] = {
    { "none",    IRQF_TRIGGER_NONE },
    { "falling", IRQF_TRIGGER_FALLING },
    { "rising",  IRQF_TRIGGER_RISING },
};

#define SYS_STR "/sys/class/gpio/"
#define CHIP_STR SYS_STR"gpiochip%d/"
#define GPIO_STR SYS_STR"gpio%d/"

#define MAX_GPIO_SIZE 32

struct gpio {
    unsigned int number;
    int fd;
};

struct gpio gpios[MAX_GPIO_SIZE] = {{0}};
unsigned int gpio_size = 0;

/* SIGNAL STUFF */
volatile sig_atomic_t shutdown_flag = 0;

void sig_handler(int signo)
{
  if (signo == SIGINT)
    shutdown_flag = 1;
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
    if(argc < 2) {
        printf("Please provide gpiochip number.\n");
        exit(EXIT_FAILURE);
    }

    unsigned int gpiochip = strtol(argv[1], 0, 10);

    char buffer[256];
    int fd;

    sprintf(buffer, CHIP_STR"base", gpiochip);
    fd = open(buffer, O_RDONLY);
    VALID_OR_DIE(fd, buffer)

    int readen = read(fd, buffer, sizeof(buffer));

    buffer[readen + 1] = '\0';
    unsigned int base = strtol(buffer, 0, 10);
    close(fd);

    sprintf(buffer, CHIP_STR"ngpio", gpiochip);
    fd = open(buffer, O_RDONLY);
    VALID_OR_DIE(fd, buffer)

    readen = read(fd , buffer, sizeof(buffer));
    buffer[readen + 1] = '\0';
    unsigned int ngpio = strtol(buffer, 0, 10);
    close(fd);

    printf("gpio_chip:\n");
    printf("\tbase: %d\n", base);
    printf("\tngpio: %d\n", ngpio);

    /* adding only gpios with interrupt enabled */
    char value;
    int i;
    for(i = 0;i < ngpio && i < MAX_GPIO_SIZE; i++)
    {
        sprintf(buffer, GPIO_STR"edge", i + base);
        fd = open(buffer, O_RDONLY);
        if(fd > 0) {
            readen = read(fd, buffer, sizeof(buffer));
            buffer[readen + 1] = '\0';
            if(strncmp(trigger_types[0].name, buffer, strlen(trigger_types[0].name)) != 0) {
                sprintf(buffer, GPIO_STR"value", i + base);
                gpios[gpio_size].number = i + base;
                gpios[gpio_size].fd = open(buffer, O_RDONLY);
                VALID_OR_DIE(fd, buffer)                
                /* rewind to clear kernfs_fop_poll */
                read(gpios[gpio_size].fd, &value, 1);
                if(lseek(gpios[gpio_size].fd, 0, SEEK_SET) == -1)
                    perror("lseek");
                printf("Added gpio %d to watchlist.\n", gpios[gpio_size].number);
                gpio_size++;
            }
        }
        close(fd);
    }

    fd_set efds;
    int maxfd = 0;
    int ready;

    if (signal(SIGINT, sig_handler) == SIG_ERR)
      perror("\ncan't catch SIGINT\n");

    printf("Entering loop with %d gpios.\n", gpio_size);

    while(shutdown_flag != 1) {
        FD_ZERO(&efds);        
        maxfd = 0;

        for(i = 0; i < gpio_size; i++)
        {
            FD_SET(gpios[i].fd, &efds);
            maxfd = (maxfd < gpios[i].fd) ? gpios[i].fd : maxfd;
        }

        ready = pselect(maxfd + 1, NULL, NULL, &efds, NULL, NULL);

        if(ready > 0)
            for(i = 0; i < gpio_size; i++)
                if(FD_ISSET(gpios[i].fd, &efds)) {
                    read(gpios[i].fd, &value, 1);
                    if(lseek(gpios[i].fd, 0, SEEK_SET) == -1)
                        perror("lseek");
                    printf("gpio number=%d interrupt caught\n", gpios[i].number);
                }
    }

    /* rewinding and closing stuff */
    printf("Gracefull exit.\n");

    for(i = 0; i < gpio_size; i++) {
        read(gpios[i].fd, &value, 1);
        if(lseek(gpios[i].fd, 0, SEEK_SET) == -1)
            perror("lseek");
        close(gpios[i].fd);
    }

    return 0;
}
