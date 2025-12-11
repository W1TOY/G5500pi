#ifndef _ADS1015_H
#define _ADS1015_H

#include <stdint.h>

extern int readADC_SingleEnded (uint8_t i2c_addr, uint16_t channel, uint16_t *data, char ynot[]);

#endif // _ADS1015_H
