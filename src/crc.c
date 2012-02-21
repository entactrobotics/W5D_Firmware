// --------------------------------------------------------------------------------------
// Module     : CRC
// Description: CRC computation
// Author     : N. El-Fata
// --------------------------------------------------------------------------------------
// Pulse Innovation, Inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------
#include <stdint.h>

#include "crc.h"

#define CRC_TABLE_MAX   256

uint8_t crc_table[CRC_TABLE_MAX];

void crc_init(void)
{
    uint8_t polynomial = 0xD5;
    uint16_t i,j,temp;

    for(i=0; i<CRC_TABLE_MAX; i++)
    {
        temp = i;
        for(j=0; j<8; j++)
        {
            if((temp & 0x80) != 0)
            {
                temp = (temp << 1) ^ polynomial;
            }
            else
            {
                temp = temp << 1;
            }
        }
        crc_table[i] = temp & 0xFF;
    }
}

uint8_t crc_compute(uint8_t *buf, uint32_t len)
{
    uint8_t crc = 0;
    uint8_t index;
    uint32_t i;

    for(i=0; i<len; i++)
    {
        index = (crc ^ buf[i]) & 0xFF;
        crc   = crc_table[index];
    }
    return(crc);
}

