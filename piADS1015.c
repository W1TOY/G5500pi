/* simple I2C interface to read an ADS1015 ADC channel.
 *
 * liberally derived from Adafruit_ADS1015.cpp, see https://github.com/adafruit/Adafruit_ADS1X15
 * ADS1015 data sheet: https://www.ti.com/lit/ds/symlink/ads1015.pdf
 *
 * #define _UNIT_TEST_MAIN to make a stand-alone test program, build and run as follows:
 *
 *   gcc -Wall -O2 -D_UNIT_TEST_MAIN piADS1015.c piI2C.c -o piADS1015
 *   ./piADS1015 0x48 1     # I2C device address, ADC channel number 0 .. 3
 *
 *   if this reports a permission error, try adding user pi to the i2c group:
 *      sudo usermod -a -G i2c pi
 */


/*!

    @file     Adafruit_ADS1015.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the ADS1015/ADS1115 ADC

    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/


#include <stdint.h>
#include <string.h>


// create an empty file if this does not appear to be on a Pi
#include "isapi.h"

#if defined(ISA_PI)


/* on a Pi! real implementation
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "piADS1015.h"
#include "piI2C.h"


// bits and addresss

#define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and set ALERT/RDY high (default)
#define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
#define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)
#define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3



#define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1015_REG_POINTER_CONFIG      (0x01)
#define ADS1015_REG_POINTER_CONVERT     (0x00)

#define ADS1015_CONVERSIONDELAY         (1)       // ms


/* read the given ADC channel in single-ended mode.
 * we use piI2C and assume piI2CInit() has already been called.
 * return 0 if ok, else return -1 with brief excuse in ynot[]
 */
int readADC_SingleEnded (uint8_t i2c_addr, uint16_t channel, uint16_t *data, char ynot[])
{
  
    // Start with default values
    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                      ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                      ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                      ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
  
    // Set gain
    config |= ADS1015_REG_CONFIG_PGA_4_096V;
  
    // Set 'start single-conversion' bit
    config |= ADS1015_REG_CONFIG_OS_SINGLE;
  
    // Set single-ended input channel
    switch (channel)
    {
    case (0):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
        break;
    case (1):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
        break;
    case (2):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
        break;
    case (3):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
        break;
    default:
        sprintf (ynot, "bogus ADC channel %d, must be 0..3", channel);
        return (-1);
    }
  
    // Write config register to the ADC
    if (piI2CWrite16 (i2c_addr, ADS1015_REG_POINTER_CONFIG, config, ynot) < 0)
        return (-1);
  
    // Wait for the conversion to complete (not worth polling)
    usleep(1000*ADS1015_CONVERSIONDELAY);
  
    // Read the conversion results
    if (piI2CRead16 (i2c_addr, ADS1015_REG_POINTER_CONVERT, data, ynot) < 0)
        return (-1);
  
    // result is in upper 12-bits for AD1015
    *data >>= 4U;

    // value is actually signed, so it can be slightly negative when near ground potential
    if (*data > 0x7FF)
        *data = 0;

    // ok
    return (0);
}

#if defined(_UNIT_TEST_MAIN)

#include <stdio.h>

int main (int ac, char *av[])
{
    if (ac != 3) {
        fprintf (stderr, "Usage: %s ADC_I2C_addr ADC_channel\n", av[0]);
        return(1);
    }

    uint8_t addr = strtol (av[1], NULL, 16);
    uint16_t channel = strtol (av[2], NULL, 0);

    char ynot[1024];

    if (piI2CInit(ynot) < 0) {
        fprintf (stderr, "piI2CInit(): %s\n", ynot);
        return(1);
    }

    uint16_t v;
    if (readADC_SingleEnded (addr, channel, &v, ynot) < 0) {
        fprintf (stderr, "readADC_SingleEnded(): %s\n", ynot);
        return(1);
    }

    printf ("%4u 0x%04x\n", v, v);

    return (0);
}

#endif // (_UNIT_TEST_MAIN)


#else // !defined(ISA_PI)


// dummy implementation when not on a Pi
int readADC_SingleEnded (uint8_t i2c_addr, uint16_t channel, uint16_t *data, char ynot[]) {
    (void) i2c_addr;
    (void) channel;
    (void) data;
    (void) ynot;
    strcpy (ynot, "readADC_SingleEnded only on RPi");
    return (-1);
}


#if defined(_UNIT_TEST_MAIN)

// dummy test
#include <stdio.h>
int main (int ac, char *av[])
{
    printf ("piADS1015 only on RPi\n");
    return (1);
}

#endif // (_UNIT_TEST_MAIN)



#endif // ISA_PI
