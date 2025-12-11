#ifndef _PI_I2C_H
#define _PI_I2C_H

#include <stdint.h>

extern int piI2CInit (char ynot[]);
extern int piI2CRead16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t *data, char ynot[]);
extern int piI2CWrite16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t data, char ynot[]);
extern void piI2CClose (void);


#endif
