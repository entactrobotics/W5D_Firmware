// --------------------------------------------------------------------------------------
// Module     : SPI
// Author     : N. El-Fata
//            : Revised by R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// Pulse Innovation inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------

#ifndef SPI_H
#define SPI_H

// comment out if mutual exclusion is not required for access to spi.c
#define SPI_MUTEX

#define SPI_BYTE_DELAY_US       (30)
#define SLAVE_DSPIC_SPEED_HZ    (10000000)

#define SPI_DEV_NAME1    "/dev/spidev1.0"    // SPI Bus 1, CS0
#define SPI_DEV_NAME2    "/dev/spidev1.1"    // SPI Bus 1, CS1

#define SPI_TRANSFERS       (1)
#define SPI_BITS_PER_WORD   (8)

#pragma pack(push)
#pragma pack(1)
typedef struct
{
    uint8_t addr;
    int32_t data;
    uint8_t crc;
} slave_packet_t;

typedef struct
{
    uint8_t crc;
    int16_t data[3];
    uint8_t extra;
} slave_burst_packet_t;
#pragma pack(pop)

#define MAXON_KT        (144)        // N.mm/A
#define W5D_K_AMP       (0.00479)   // A/count (spec'd @ 4.79mA/count)

int spi_init(void);
int spi_uninit(void);

int spi_torque_command(float *tau);

#endif // SPI_H
