// --------------------------------------------------------------------------------------
// Module     : MEM
// Description: Memory mapped IO access
// Author     : R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include "mem.h"

static int      mem_fd          = -1;
static uint32_t mem_map_size    = 0;
static uint32_t mem_map_mask    = 0;

#ifdef MEM_MUTEX
static pthread_mutex_t mem_mutex    = PTHREAD_MUTEX_INITIALIZER;
#endif

// page pointers
volatile void *fpga_page = NULL;
volatile void *gpio2_page = NULL;   // using gpios (70 thru 77 of GPIO2)
volatile void *gpio4_page = NULL;   // using gpios (144 and 145 of GPIO4)

// register pointers
volatile int32_t *encoder_0 = NULL;
volatile int32_t *clr_encoder_0 = NULL;
volatile int32_t *gpio2_OE = NULL;
volatile int32_t *gpio2_SDO = NULL;
volatile int32_t *gpio2_CDO = NULL;
volatile int32_t *gpio2_DI = NULL;
volatile int32_t *gpio4_OE = NULL;
volatile int32_t *gpio4_SDO = NULL;
volatile int32_t *gpio4_CDO = NULL;
volatile int32_t *gpio4_DI = NULL;


int mem_slave_init(void);   // called by mem_init() to set slave reset lines as outputs

int mem_init(void)
{
    if(mem_fd >= 0) return(0); // memory has already been initialized (nothing else to do)
    
    // open a file descriptor to /dev/mem
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(mem_fd < 0)
    {
        fprintf(stderr, "ERR: /dev/mem open failed: %s\n", strerror(errno));
        return(-1);
    }

    // page size and mask
    mem_map_size = sysconf(_SC_PAGE_SIZE);
    mem_map_mask = mem_map_size - 1;
    
    // mapping FPGA (encoder) registers
    fpga_page = mmap(0, mem_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, QEI_BASE&~mem_map_mask);
    if(fpga_page == MAP_FAILED)
    {
        fprintf(stderr, "ERR: mmap failed: %s\n", strerror(errno));
        return(-1);
    }
    encoder_0 = (volatile int32_t *) ((uint32_t)fpga_page + (QEI_BASE&mem_map_mask));
    clr_encoder_0 = (volatile int32_t *) ((uint32_t)fpga_page + (CLR_QEI_BASE&mem_map_mask));
    
    // mapping GPIO2 (gpio's 64 to 95) registers
    gpio2_page = mmap(0, mem_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, OMAP34XX_GPIO3_BASE&~mem_map_mask);
    if(gpio2_page == MAP_FAILED)
    {
        fprintf(stderr, "ERR: mmap failed: %s\n", strerror(errno));
        return(-1);
    }
    gpio2_OE = (volatile int32_t *) ((uint32_t)gpio2_page + (OMAP24XX_GPIO_OE&mem_map_mask));
    gpio2_SDO = (volatile int32_t *) ((uint32_t)gpio2_page + (OMAP24XX_GPIO_SETDATAOUT&mem_map_mask));
    gpio2_CDO = (volatile int32_t *) ((uint32_t)gpio2_page + (OMAP24XX_GPIO_CLEARDATAOUT&mem_map_mask));
    gpio2_DI = (volatile int32_t *) ((uint32_t)gpio2_page + (OMAP24XX_GPIO_DATAIN&mem_map_mask));
    
    // mapping GPIO4 (gpio's 128 to 159) registers
    gpio4_page = mmap(0, mem_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, OMAP34XX_GPIO5_BASE&~mem_map_mask);
    if(gpio4_page == MAP_FAILED)
    {
        fprintf(stderr, "ERR: mmap failed: %s\n", strerror(errno));
        return(-1);
    }
    gpio4_OE = (volatile int32_t *) ((uint32_t)gpio4_page + (OMAP24XX_GPIO_OE&mem_map_mask));
    gpio4_SDO = (volatile int32_t *) ((uint32_t)gpio4_page + (OMAP24XX_GPIO_SETDATAOUT&mem_map_mask));
    gpio4_CDO = (volatile int32_t *) ((uint32_t)gpio4_page + (OMAP24XX_GPIO_CLEARDATAOUT&mem_map_mask));
    gpio4_DI = (volatile int32_t *) ((uint32_t)gpio4_page + (OMAP24XX_GPIO_DATAIN&mem_map_mask));

    // initializing mutexes
    #ifdef MEM_MUTEX
    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_settype(&mutexAttr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&mem_mutex, &mutexAttr);
    #endif

    // setting slave reset lines as outputs
    mem_slave_init();

    return(0);
}

int mem_uninit(void)
{
    int rc;
    //void *page1 = fpga_page;
    //void *page2 = gpio2_page;
    //void *page3 = gpio4_page;
    
    if (mem_fd==-1) return(0);

    // unmap
    rc = munmap(fpga_page, mem_map_size);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: munmap failed: %s\n", strerror(errno));
        return(-1);
    }
    rc = munmap(gpio2_page, mem_map_size);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: munmap failed: %s\n", strerror(errno));
        return(-1);
    }
    rc = munmap(gpio4_page, mem_map_size);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: munmap failed: %s\n", strerror(errno));
        return(-1);
    }
    
    // close /dev/mem
    rc = close(mem_fd);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: close failed: %s\n", strerror(errno));
        return(-1);
    }

    mem_fd  = -1;
    fpga_page = NULL;
    gpio2_page = NULL;
    gpio4_page = NULL;
    
    return(0);
}

int mem_encoder_read_all(int32_t *enc)
{
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    memcpy((void *)enc, (void *)encoder_0, 8*sizeof(int32_t));
    
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_encoder_clear_all(int32_t *enc)
{
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    // reading from the clear registers to reset the counters
    memcpy((void *)enc, (void *)clr_encoder_0, 8*sizeof(int32_t));
    
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_slave_init(void)
{
    int32_t oe_reg = *gpio4_OE;
    int32_t gpio_out = ~((1<<(GPIO_PIC1_RST&0x1F))|(1<<(GPIO_PIC2_RST&0x1F)));
    int32_t l = oe_reg&gpio_out;
    
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    // setting gpio's associated with dspic reset (144,145) as outputs
    // 0 = output, 1 = input
    *gpio4_OE = l;
    
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_slave_disable(void)
{
    int32_t dspic1_rst = GPIO_PIC1_RST&0x1F;
    int32_t dspic2_rst = GPIO_PIC2_RST&0x1F;
    int32_t l = (1<<dspic1_rst)|(1<<dspic2_rst);
    
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    // setting dspic reset lines to high
    *gpio4_SDO = l;

    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_slave_enable(void)
{
    int32_t dspic1_rst = GPIO_PIC1_RST&0x1F;
    int32_t dspic2_rst = GPIO_PIC2_RST&0x1F;
    int32_t l = (1<<dspic1_rst)|(1<<dspic2_rst);
    
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    // setting dspic reset lines to low
    *gpio4_CDO = l;
    
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_led_init(void)
{
    int32_t oe_reg = *gpio2_OE;
    int32_t gpio_out = ~((1<<(GPIO_GPOUT_0&0x1F))|(1<<(GPIO_GPOUT_1&0x1F))|(1<<(GPIO_GPOUT_2&0x1F))|(1<<(GPIO_GPOUT_3&0x1F))|(1<<(GPIO_GPOUT_4&0x1F))|(1<<(GPIO_GPOUT_5&0x1F)));
    int32_t l = oe_reg&gpio_out;
    
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    // setting gpio's associated with the LED as outputs
    // 0 = output, 1 = input
    *gpio2_OE = l;
    //printf("OE is set to %d \n", l);
    
    
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    return (0);
}

int mem_led_set(int value)
{
    int32_t l = (1<<(GPIO_GPOUT_0&0x1F));
    
    if (mem_fd==-1) return(-1);
    
    // Critical section start
    #ifdef MEM_MUTEX
    if (pthread_mutex_lock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    if (value) {*gpio2_SDO = l;}
    else {*gpio2_CDO = l;}
    //printf("SDO is set to %d \n", l); 
        
    // Critical section end
    #ifdef MEM_MUTEX
    if (pthread_mutex_unlock(&mem_mutex) != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif    
        
    return (0);
}