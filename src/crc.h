// --------------------------------------------------------------------------------------
// Module     : CRC
// Author     : N. El-Fata
// --------------------------------------------------------------------------------------
// Pulse Innovation, Inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------

#ifndef CRC_H
#define CRC_H

void crc_init(void);
uint8_t crc_compute(uint8_t *buf, uint32_t len);

#endif // CRC_H
