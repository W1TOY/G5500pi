/* Simple I2C implementation for Raspberry Pi running Debian linux.
 * Compiles on any UNIX but functions all return failure if not above.
 * linux info: https://www.kernel.org/doc/Documentation/i2c/dev-interface
 */


#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "isapi.h"
#include "piI2C.h"


#if defined(ISA_PI)

/* real implementation
 */

#include <sys/ioctl.h>
#include <sys/file.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>



// set to 1 for more status reporting
static int verbose = 0;

// file access and last bus address
static int i2c_fd = -1;
static int i2c_addr = -1;


/* insure system is set to the given I2C bus addr.
 * harmless if called more than once with same addr.
 * return 0 if ok, else -1 with brief excuse in ynot
 */
static int piI2CSetBusAddr (uint8_t bus_addr, char ynot[])
{
        // ignore if same as previous
        if (i2c_addr == bus_addr)
            return (0);


        if (ioctl(i2c_fd, I2C_SLAVE, bus_addr) < 0) {
            sprintf (ynot, "I2C: %s(0x%02X): %s", __func__, bus_addr, strerror(errno));
            return (-1);
        } else {
            // save
            i2c_addr = bus_addr;
            if (verbose)
                fprintf (stderr, "I2C: %s (0x%02X) ok\n", __func__, bus_addr);
            return (0);
        }
}

/* initialize.
 * return 0 if ok else -1 with brief excuse in ynot.
 * harmless if called more than once.
 */
int piI2CInit (char ynot[])
{
        // open the I2C driver
	static const char filename[] = "/dev/i2c-1";
        if (i2c_fd < 0) {
            i2c_fd = open(filename, O_RDWR);
            if (i2c_fd < 0) {
                sprintf (ynot, "I2C: %s(): %s: %s", __func__, filename, strerror(errno));
            } else if (verbose) {
                fprintf (stderr, "I2C: %s(): %s open ok\n", __func__, filename);
            }
        }

        // return whether success
        return (i2c_fd >= 0 ? 0 : -1);
}

/* read a 16 bit word from the given device register at the given bus address.
 * return 0 if ok else -1 with brief excuse in ynot
 */
int piI2CRead16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t *data, char ynot[])
{
        // set bus address
        if (piI2CSetBusAddr (bus_addr, ynot) < 0)
            return (-1);

        // send the register then read two bytes
        uint8_t r8 = dev_reg;
        uint8_t b8[2];
        if (write (i2c_fd, &r8, 1) != 1 || read (i2c_fd, b8, 2) != 2) {
            sprintf (ynot, "%s (0x%02x, 0x%02x): %s\n", __func__, bus_addr, dev_reg, strerror(errno));
            return (-1);
        }

        // combine bytes into word, big endian
        *data = (b8[0] << 8) | b8[1];

        if (verbose)
            fprintf (stderr, "I2C: %s (0x%02x, 0x%02x): %u 0x%02x\n", __func__, bus_addr, dev_reg,
                                                                                *data, *data);

        // ok
        return (0);
}

/* write a 16 bit word to the given device register at the given bus address.
 * return 0 if ok else -1 with brief excuse in ynot
 */
int piI2CWrite16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t data, char ynot[])
{
        // set bus address
        if (piI2CSetBusAddr (bus_addr, ynot) < 0)
            return (-1);

        // send the register then two bytes of data
        uint8_t rd[3];
        rd[0] = dev_reg;
        rd[1] = (data >> 8) & 0xff;
        rd[2] = data & 0xff;
        if (write (i2c_fd, rd, 3) != 3) {
            sprintf (ynot, "%s (0x%02x, 0x%02x): %s\n", __func__, bus_addr, dev_reg, strerror(errno));
            return (-1);
        }

        if (verbose)
            fprintf (stderr, "I2C: %s (0x%02x, 0x%02x): %u 0x%02x\n", __func__, bus_addr, dev_reg,
                                                                                data, data);

        // ok
        return (0);
}

/* insure i2c handle is closed
 */
void piI2CClose()
{
        if (i2c_fd >= 0) {
            if (verbose)
                fprintf (stderr, "I2C: %s()\n", __func__);
            close (i2c_fd);
            i2c_fd = -1;
        }
}

#else



/* empty implementation
 */

int piI2CInit (char ynot[])
{
    strcpy (ynot, "piI2C only on RPi");
    return (-1);
}
int piI2CRead16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t *data, char ynot[])
{
    (void) bus_addr;
    (void) dev_reg;
    (void) data;

    strcpy (ynot, "piI2C only on RPi");
    return (-1);
}
int piI2CWrite16 (uint8_t bus_addr, uint8_t dev_reg, uint16_t data, char ynot[])
{
    (void) bus_addr;
    (void) dev_reg;
    (void) data;

    strcpy (ynot, "piI2C only on RPi");
    return (-1);
}
void piI2CClose (void)
{
}


#endif

