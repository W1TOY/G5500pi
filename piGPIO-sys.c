/* Simple GPIO implementation for Raspberry Pi running Debian linux.
 * Compiles on any UNIX but functions all return failure if not above.
 *
 * All pin numbers refer to BCM GPIO numbers. Run pinout to see where they on the header.
 *
 * #define _UNIT_TEST_MAIN to make a stand-alone test program, build and run as follows:
 *
 *   gcc -Wall -O2 -D_UNIT_TEST_MAIN piGPIO-sys.c -o piGPIO-sys
 *   ./piGPIO-sys 20 1     # set pin 20 as output with state hi
 *   ./piGPIO-sys 20       # set pin 20 as input and read
 *
 *   if this reports a permission error, try adding user pi to the gpio group:
 *      sudo usermod -a -G gpio pi
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "isapi.h"
#include "piGPIO.h"


#if defined(ISA_PI)


/* real implementation
 */

static const char base_path[] = "/sys/class/gpio";

static int exportPin (int p, char ynot[])
{
        char path[100];
        FILE *fp;

        // check if already exported
        snprintf (path, sizeof(path), "%s/gpio%d", base_path, p);
        fp = fopen (path, "r");
        if (fp) {
            fclose (fp);
            return (0);
        }

        // nope, export
        snprintf (path, sizeof(path), "%s/export", base_path);
        fp = fopen (path, "w");
        if (!fp) {
            if (ynot)
                strcpy (ynot, strerror(errno));
            return (-1);
        }
        fprintf (fp, "%d\n", p);
        fclose(fp);

        // confirm
        snprintf (path, sizeof(path), "%s/gpio%d", base_path, p);
        fp = fopen (path, "r");
        if (!fp) {
            if (ynot)
                strcpy (ynot, strerror(errno));
            return (-1);
        }
        fclose(fp);

        // ok
        if (verbose)
            fprintf (stderr, "pin %d exported ok\n", p);
        return (0);
}

static void setPinDirection (int p, int in_else_out)
{
        char path[100];
        FILE *fp;

        snprintf (path, sizeof(path), "%s/gpio%d/direction", base_path, p);
        fp = fopen (path, "w");
        if (!fp) {
            if (exportPin (p, NULL) < 0)
                return;
            fp = fopen (path, "w");
        }
        if (!fp)
            return;
        fprintf (fp, "%s\n", in_else_out ? "in" : "out");
        fclose(fp);
}

static void setPinState (int p, int one)
{
        char path[100];
        FILE *fp;

        snprintf (path, sizeof(path), "%s/gpio%d/value", base_path, p);
        fp = fopen (path, "w");
        if (!fp) {
            if (exportPin (p, NULL) < 0)
                return;
            fp = fopen (path, "w");
        }
        if (!fp)
            return;
        fprintf (fp, "%d\n", one ? 1 : 0);
        fclose(fp);
}

static int readPin (int p)
{
        char path[100];
        char valu[10];
        FILE *fp;
        int result = 0;

        snprintf (path, sizeof(path), "%s/gpio%d/value", base_path, p);
        fp = fopen (path, "r");
        if (!fp) {
            if (exportPin (p, NULL) < 0)
                return (result);
            fp = fopen (path, "r");
        }
        if (!fp)
            return (result);

        if (fgets (valu, sizeof(valu), fp))
            result = atoi (valu);
        fclose(fp);

        return (result);
}



/* initialize.
 * return 0 if ok else -1 with brief excuse in ynot.
 * harmless if called more than once.
 */
int piGPIOInit (char ynot[])
{
        return (exportPin(1, ynot));
}


/* set the given pin as input with pullup
 */
void piGPIOsetAsInput (uint8_t p)
{
        setPinDirection (p, 1);
}

/* set the given pin as output
 */
void piGPIOsetAsOutput (uint8_t p)
{
        setPinDirection (p, 0);
}

/* set the given pin HI
 */
void piGPIOsetHi (uint8_t p)
{
        setPinState (p, 1);
}

/* set the given pin LOW
 */
void piGPIOsetLo (uint8_t p)
{
        setPinState (p, 0);
}

/* set the given pin hi or lo
 */
void piGPIOsetHiLo (uint8_t p, int hi)
{
        setPinState (p, hi);
}

/* return 0 or 1 depending on whether the given pin is hi or lo
 */
int piGPIOreadPin (uint8_t p)
{
        return (readPin (p));
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
            printf ("pin %d -> %d\n", pin, state);

        } else if (ac == 3) {
            // looks like a write
            int pin = atoi(av[1]);
            int state = atoi(av[2]) == 0 ? 0 : 1;
            if (piGPIOInit(ynot) < 0) {
                fprintf (stderr, "piGPIOInit(): %s\n", ynot);
                return (1);
            }
            piGPIOsetAsOutput (pin);
            piGPIOsetHiLo (pin, state);
            printf ("pin %d <- %d\n", pin, state);

        } else {
            fprintf (stderr, "%s: [-v] BCM_pin [state]\n", av[0]);
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

