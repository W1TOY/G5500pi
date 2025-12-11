#ifndef _PI_GPIO_H
#define _PI_GPIO_H

#include <stdint.h>

extern int piGPIOInit (char ynot[]);
extern void piGPIOsetAsInput (uint8_t p);
extern void piGPIOsetAsOutput (uint8_t p);
extern void piGPIOsetHi (uint8_t p);
extern void piGPIOsetLo (uint8_t p);
extern void piGPIOsetHiLo (uint8_t p, int hi);
extern int piGPIOreadPin (uint8_t p);
extern int verbose;

#endif
