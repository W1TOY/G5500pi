/* Simple GPIO implementation for Raspberry Pi running Debian linux.
 * Compiles on any UNIX but functions all return failure if not above.
 *
 * All pin numbers refer to BCM GPIO numbers. Run pinout to see where they on the header.
 *
 * #define _UNIT_TEST_MAIN to make a stand-alone test program, build and run as follows:
 *
 *   gcc -Wall -O2 -D_UNIT_TEST_MAIN piGPIO.c -o piGPIO
 *   ./piGPIO 20 1     # set pin 20 as output with state hi
 *   ./piGPIO 20       # set pin 20 as input and read
 *
 *   if this reports a permission error, try adding user pi to the gpio group:
 *      sudo usermod -a -G gpio pi
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "isapi.h"
#include "piGPIO.h"


#if defined(ISA_PI)


/* real implementation
 */


#include <sys/mman.h>
#include <sys/file.h>


// mapped base of GPIO control registers
static volatile uint32_t *gbase;

// handy way to find selection make from pin and mode
static inline uint32_t GPIO_SEL_MASK (uint8_t p, uint32_t m) {
        return (m<<(3*(p%10)));
}



/* set gbase so it points to the physical address of the GPIO controller.
 * return 0 if ok, else -1 with brief excuse in ynot[].
 */
static int mapGPIOAddress(char ynot[])
{
        // acquire exclusive access to kernel physical address
        static const char gpiofilefile[] = "/dev/gpiomem";
        int fd = open (gpiofilefile, O_RDWR|O_SYNC);
        if (fd < 0) {
            sprintf (ynot, "%s: %s", gpiofilefile, strerror(errno));
            return (-1);
        }

        // mmap access
        gbase = (uint32_t *) mmap(NULL, 0xB4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

        // fd not needed after setting up mmap
        close(fd);

        // check for error, set gbase NULL if so
        if (gbase == MAP_FAILED) {
            gbase = NULL;
            sprintf (ynot, "mmap(): %s", strerror(errno));
            return (-1);
        }

        if (verbose)
            fprintf (stderr, "GPIO: %s() ok\n", __func__);

        // worked
        return (0);
}


/* initialize.
 * return 0 if ok else -1 with brief excuse in ynot.
 * harmless if called more than once.
 */
int piGPIOInit (char ynot[])
{
        if (!gbase)
            return (mapGPIOAddress (ynot));
        if (verbose)
            fprintf (stderr, "GPIO: %s() ok\n", __func__);
        return (0);
}


/* set the given pin as input with pullup
 */
void piGPIOsetAsInput (uint8_t p)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s() not ready\n", __func__);
            return;
        }

        // set as input
        gbase[p/10] &= ~GPIO_SEL_MASK(p,7);

        // enable pullup -- BCM2835
        gbase[37] = 2;
        gbase[38+p/32] = 1UL << (p%32);
        gbase[37] = 0;
        gbase[38+p/32] = 0;

        // enable pullup -- BCM2711
        gbase[57+p/16] = (gbase[57+p/16] & ~(3UL << 2*(p%16))) | (1UL << 2*(p%16));

        if (verbose)
            fprintf (stderr, "GPIO: %s (%d) ok\n", __func__, p);
}

/* set the given pin as output
 */
void piGPIOsetAsOutput (uint8_t p)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s(%d) not ready\n", __func__, p);
            return;
        }

        gbase[p/10] = (gbase[p/10] & ~GPIO_SEL_MASK(p,7)) | GPIO_SEL_MASK(p,1);

        if (verbose)
            fprintf (stderr, "GPIO: %s (%d) ok\n", __func__, p);
}

/* set the given pin HI
 */
void piGPIOsetHi (uint8_t p)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s(%d) not ready\n", __func__, p);
            return;
        }

        gbase[7+p/32] = 1UL << (p%32);

//        if (verbose)
//            fprintf (stderr, "GPIO: %s (%d) ok\n", __func__, p);
}

/* set the given pin LOW
 */
void piGPIOsetLo (uint8_t p)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s(%d) not ready\n", __func__, p);
            return;
        }

        gbase[10+p/32] = 1UL << (p%32);

//        if (verbose)
//            fprintf (stderr, "GPIO: %s (%d) ok\n", __func__, p);
}

/* set the given pin hi or lo
 */
void piGPIOsetHiLo (uint8_t p, int hi)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s (%d, %d) not ready\n", __func__, p, hi);
            return;
        }

        if (hi)
            piGPIOsetHi (p);
        else
            piGPIOsetLo (p);
}

/* return 0 or 1 depending on whether the given pin is hi or lo
 */
int piGPIOreadPin (uint8_t p)
{
        if (!gbase) {
            fprintf (stderr, "GPIO: %s(%d) not ready\n", __func__, p);
            return(0);
        }

        int state = (gbase[13+p/32] & (1UL<<(p%32))) != 0;

        if (verbose)
            fprintf (stderr, "GPIO: %s (%d) %d\n", __func__, p, state);

        return (state);
}


#if defined(_UNIT_TEST_MAIN)

int verbose;

int main (int ac, char *av[])
{
        char ynot[1024];

        if (ac > 1 && strcmp (av[1], "-v") == 0) {
            verbose = 1;
            ac -= 1;
            av += 1;
        }

        if (ac == 2) {
            // looks like a read
            int pin = atoi(av[1]);
            if (piGPIOInit(ynot) < 0) {
                fprintf (stderr, "piGPIOInit(): %s\n", ynot);
                return (1);
            }
            piGPIOsetAsInput (pin);
            int state = piGPIOreadPin (pin);
            printf ("%d %d\n", pin, state);

        } else if (ac == 3) {
            // looks like a write
            int pin = atoi(av[1]);
            int state = atoi(av[2]);
            if (piGPIOInit(ynot) < 0) {
                fprintf (stderr, "piGPIOInit(): %s\n", ynot);
                return (1);
            }
            piGPIOsetAsOutput (pin);
            piGPIOsetHiLo (pin, state);

        } else {
            fprintf (stderr, "%s: BCM_pin [0/1]\n", av[0]);
            return (1);
        }

        return (0);
}

#endif // _UNIT_TEST_MAIN


#else

/* not a pi -- just provide dummy implementations
 */


int piGPIOInit (char ynot[])
{
    strcpy (ynot, "piGPIO only on RPi");
    return (-1);
}

void piGPIOsetAsInput (uint8_t p)
{
    (void) p;
}

void piGPIOsetAsOutput (uint8_t p)
{
    (void) p;
}

void piGPIOsetHi (uint8_t p)
{
    (void) p;
}

void piGPIOsetLo (uint8_t p)
{
    (void) p;
}

void piGPIOsetHiLo (uint8_t p, int hi)
{
    (void) p;
    (void) hi;
}

int piGPIOreadPin (uint8_t p)
{
    (void) p;
    return (0);
}



#if defined(_UNIT_TEST_MAIN)

int main (int ac, char *av[])
{
    printf ("piGPIO only on RPi\n");
    return (1);
}

#endif // _UNIT_TEST_MAIN


#endif // ISA_PI

