// --------------------------------------------------------------------------------------
// Module     : SPI
// Description: SPI access functions
// Author     : N. El-Fata
//            : Revised by R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// Pulse Innovation inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------
// References:
// http://www.kernel.org/doc/Documentation/spi/spidev

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>

#include "spi.h"
#include "crc.h"

static int spi_fd1 = -1;
static int spi_fd2 = -1;

#ifdef SPI_MUTEX
static pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

int spi_burst_transfer(int fd, uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len);

int spi_init()
{
    int rc;
	int speed_hz = SLAVE_DSPIC_SPEED_HZ;
    
    crc_init();
    
    // SPI Bus 1, CS0 
    spi_fd1 = open(SPI_DEV_NAME1, O_RDWR);
    if(spi_fd1 < 0)
    {
        fprintf(stderr, "ERR: open failed on device at CS0\n");
        return(-1);
    }

    // SPI Bus 1, CS1 
    spi_fd2 = open(SPI_DEV_NAME2, O_RDWR);
    if(spi_fd2 < 0)
    {
        fprintf(stderr, "ERR: open failed on device at CS1\n");
        return(-1);
    }
   
	// Speed Set Bus 1, CS0
	rc = ioctl(spi_fd1, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
    if(rc < 0)
    {
        fprintf(stderr, "ERR: spi_speed_set ioctl failed \n");
        return(-1);
    }
    
	// Speed Set Bus 1, CS1
    rc = ioctl(spi_fd2, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
    if(rc < 0)
    {
        fprintf(stderr, "ERR: spi_speed_set ioctl failed \n");
        return(-1);
    }

    #ifdef SPI_MUTEX
    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_settype(&mutexAttr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&spi_mutex, &mutexAttr);
    #endif

    return(0);
}

int spi_uninit()
{
    int rc = 0;
    
    // close SPI Bus 1, CS0 
    rc = close(spi_fd1);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: close failed: %s\n", strerror(errno));
        return(-1);
    }
    
    // close SPI Bus 1, CS1 
    rc = close(spi_fd2);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: close failed: %s\n", strerror(errno));
        return(-1);
    }
    
    spi_fd1 = -1;
    spi_fd2 = -1;
    
    return (0);
}

int spi_torque_command(float *tau)
{
    static slave_burst_packet_t pb_rx, pb_tx1, pb_tx2;

    // fill burst packet # 1
    pb_tx1.data[0] = (int16_t) (tau[0]/MAXON_KT/W5D_K_AMP); // copy data into packets data member
    pb_tx1.data[1] = (int16_t) (tau[1]/MAXON_KT/W5D_K_AMP);
    pb_tx1.data[2] = (int16_t) (tau[2]/MAXON_KT/W5D_K_AMP);							
    pb_tx1.crc = (crc_compute((uint8_t *)&pb_tx1.data[0], sizeof(pb_tx1.data))|0xC0); // compute crc on the data (Format: write flag | burst flag | 6-bit CRC)                                                   

    // fill burst packet # 2
    pb_tx2.data[0] = (int16_t) (tau[3]/MAXON_KT/W5D_K_AMP);
    pb_tx2.data[1] = (int16_t) (tau[4]/MAXON_KT/W5D_K_AMP);
    pb_tx2.data[2] = (int16_t) (tau[5]/MAXON_KT/W5D_K_AMP);								
    pb_tx2.crc = (crc_compute((uint8_t *)&pb_tx2.data[0], sizeof(pb_tx2.data))|0xC0);

    // transfer torque commands to slave controllers
    spi_burst_transfer(spi_fd1, (uint8_t *) &pb_tx1, (uint8_t *)&pb_rx, sizeof(slave_burst_packet_t));
    spi_burst_transfer(spi_fd2, (uint8_t *) &pb_tx2, (uint8_t *)&pb_rx, sizeof(slave_burst_packet_t));

    return(0);
}


int spi_burst_transfer(int fd, uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len)
{
    int                     rc, i;
    uint8_t                 dummy = 0;
    static struct spi_ioc_transfer xfer[SPI_TRANSFERS];
    
    #ifdef SPI_MUTEX
    int status;
    #endif
    
    if (fd==-1) return(-1);
        
    for(i=0; i<SPI_TRANSFERS; i++)
    {
        xfer[i].len             = len;
        xfer[i].tx_buf          = (unsigned long)&tx_buf[i*SPI_BITS_PER_WORD/8];
        xfer[i].rx_buf          = (unsigned long)&rx_buf[i*SPI_BITS_PER_WORD/8];
        xfer[i].speed_hz        = SLAVE_DSPIC_SPEED_HZ; //speed_hz;
        xfer[i].delay_usecs     = 0;
        xfer[i].bits_per_word   = SPI_BITS_PER_WORD;
        xfer[i].cs_change       = 0;
        xfer[i].pad             = 0;
    }

    // Critical section start
    #ifdef SPI_MUTEX
    status = pthread_mutex_lock(&spi_mutex);
    if(status != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
   
    rc = ioctl(fd, SPI_IOC_MESSAGE(SPI_TRANSFERS), &xfer[0]);

    // Critical section end
    #ifdef SPI_MUTEX
    status = pthread_mutex_unlock(&spi_mutex);
    if(status != 0)
    {
        fprintf(stderr, "ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }
    #endif
    
    if(rc < 0)
    {
        fprintf(stderr, "ERR: ioctl failed: %s\n",  strerror(errno));
        return(-1);
    }
    if(rc != len)
    {
        fprintf(stderr, "ERR: ioctl unexpected buffer size: %d\n", len + 1);
        return(-1);
    }

    // the first byte read is useless, so move all bytes to the left by one
    memmove(&rx_buf[0], &rx_buf[1], len - 1);
    // add the CRC at the end
    rx_buf[len - 1] = dummy;

    return(0);
}


