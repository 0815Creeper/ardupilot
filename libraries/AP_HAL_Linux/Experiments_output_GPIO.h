#pragma once


#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#define GPIO_BASE_PHYS 0xFE200000
#define BLOCK_SIZE     (4*1024)

//static volatile uint32_t* gpio_map = nullptr;

#define INIT_GPIO() do { \
    if (!gpio_map) { \
        int fd = open("/dev/mem", O_RDWR | O_SYNC); \
        if (fd < 0) { while(1)perror("open /dev/mem"); } \
        gpio_map = (volatile uint32_t*) mmap( \
            nullptr, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_PHYS); \
        close(fd); \
        if (gpio_map == MAP_FAILED) { while(1)perror("mmap"); } \
    } \
} while(0)

#define GPIO_SET_OUTPUT_26() do { \
    int reg = (26)/10; \
    int shift = ((26)%10)*3; \
    gpio_map[reg] &= ~(7 << shift); \
    gpio_map[reg] |=  (1 << shift); \
} while(0)

#define GPIO_WRITE_26_0() do { \
    gpio_map[10] = (1 << (26)); /* GPCLR0 */ \
} while(0)

#define GPIO_WRITE_26_1() do { \
    gpio_map[7] = (1 << (26)); /* GPSET0 */ \
} while(0)
 
