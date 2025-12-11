/* This is a Hamlib backend for the Yaesu G5500 rotator connected to Raspberry Pi with I2C ADC and GPIO.
 * Users please see the README and G5500-RPi.png schedmatic for installation details.
 *
 * Notes to developers:
 *
 * A separate thread is used for all monitor and control functions in order that the mount can be safely
 * managed between hamlib commands. ONLY THIS THREAD MAY PERFORM PHYSICAL GPIO AND I2C COMMANDS. The main
 * thread communicates with the control thread using shared variables and accessor functions.
 *
 * If the controller thread encounters an error, it sets g5500_thread_state to an error code. This causes the
 * controller thread to maintain the rotator in a stopped condition until it is cleared by the main thread.
 * An error code also causes the main thread to return a corresponding unique ROT_* number in response to
 * all get_pos requests until the error is cleared. The error is cleared upon receipt of any hamlib API
 * call that causes motion. In this way, client applications receive a persistent indication of trouble
 * to their frequenct get_pos calls and yet the operator is afforded a means to reset the error when/if
 * they decide to try again.
 *
 * In order to convert between axis ADC readings and true az and el, the driver must perform a calibration
 * sequence. This consists of sweeping each axis through its full range of motion and recording the ADC
 * values at each limit. These are stored in a file $HOME/.hamlib_g5500_cal.txt. All hamlib API calls 
 * that require this conversion will automatically commence this procedure if it can not find this file,
 * during which time it and all subsequent calls will return RIG_BUSBUSY until the procedure is complete.
 *
 * We offer a simulation mode suitable for developing client applications without hardware. This mode may
 * be activated on an RPi by setting the "simulator" configuration parameter. This can be done on the
 * rotctld command line with --set-conf=simulator=n or programmatically using the +\set_conf command.
 * The value of n = 0 means no simulation (real PI hardware required), 1 = az only, 2 = az + el to 90 and
 * 3 = az + el to 180. Mode 1 is always automatically engaged by default when built on any system that does
 * not self-identify as a RPi (see isapi.h).
 *
 *
 *************************************************************************************************************
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *************************************************************************************************************
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>


// use normal hamlib includes unless building stand-alone
#if defined(STANDALONE_G5500)
#include "g5500_sa.h"
#else
#include "hamlib/rotator.h"
#include "register.h"
#endif


/* define ISA_PI if this appears to be a Raspberry Pi
 */
#include "isapi.h"


/* our IO libs
 */
#include "piGPIO.h"
#include "piI2C.h"
#include "piADS1015.h"



/***********************************************************************************************************
 *
 *
 * configuration constants and state variables
 *
 *
 ***********************************************************************************************************/

/* ADS1015 ADC channel and I2C configuration
 */
#define ADC_CHANNEL_AZ          0               // ADS1015 channel for az
#define ADC_CHANNEL_EL          1               // ADS1015 channel for el
#define ADC_CHANNEL_POK         2               // ADS1015 channel for power ok
#define ADC_I2C_ADDR            0x48            // bus addr after r/w bit shift
#define ADC_MIN_POK             1000            // minimum ADC when power ok
#define ADC_AZ_DEADBAND         50              // 
#define ADC_EL_DEADBAND         50              // roughly +-few deg


/* RPi GPIO output pins, BCM numbering, active-hi.
 */
#define PIN_AZ_CW               25              // header 22
#define PIN_AZ_CCW              8               // header 24
#define PIN_EL_UP               7               // header 26
#define PIN_EL_DOWN             1               // header 28


/* mapping of our errors to published RIG_* errors.
 * the correspondence is rather arbitrary for some cases.
 */
#define G5500_RIG_OK            RIG_OK
#define G5500_RIG_CALIBRATING   (-RIG_BUSBUSY)
#define G5500_RIG_ERR_ADC       (-RIG_EPROTO)
#define G5500_RIG_ERR_NOPOWER   (-RIG_ENAVAIL)
#define G5500_RIG_ERR_STUCK     (-RIG_ENTARGET)
#define G5500_RIG_ERR_GPIO      (-RIG_BUSERROR)
#define G5500_RIG_ERR_INTERNAL  (-RIG_EINTERNAL)
#define G5500_RIG_ERR_BADARGS   (-RIG_EINVAL)


/* handy pseudonyms for digital line states
 */
#define PIN_ACTIVE      1
#define PIN_IDLE        0


/* basename of file in which calibration constants are stored
 */
static const char g5500_cal_file_name[] = ".hamlib_g5500_cal.txt";


/* max physical travel ranges, in degrees.
 */
#define AZ_MOUNT_MIN            0.0
#define AZ_MOUNT_MAX            450.0
#define AZ_MOUNT_WRAP           360.0
#define EL_MOUNT_MIN            0.0
#define EL_MOUNT_MAX            180.0
static float el_mount_max = EL_MOUNT_MAX;             // might change depending on simulation mode


/* desired park position, degs
 */
#define AZ_MOUNT_PARK           0.0
#define EL_MOUNT_PARK           0.0


/* tokens for our configuration parameters
 * N.B. 0 triggers a bug which sets its value from any undefined parameter
 */
enum {
    TOK_SIMULATOR = 1,          // avoid 0
};


/* calibration constants and whether they are valid
 */
static uint16_t ADC_az_min, ADC_az_max;
static uint16_t ADC_el_min, ADC_el_max;
static int ADC_cal_ok = 0;



/* simulator parameters
 */
typedef enum {
    SIM_OFF,
    SIM_AZONLY,
    SIM_EL90,
    SIM_EL180
} SimType;
static volatile SimType g5500_sim_mode; // whether and how to simulate
#define AZ_SIM_SPEED            10      // degs/sec
#define EL_SIM_SPEED            5       // degs/sec
#define AZ_SIM_MAX_ADC          2000    // simulated ADC value when at max az, not critical
#define EL_SIM_MAX_ADC          2000    // simulated ADC value when at max el, not critical



/* these state variables are used both by the main thread and the controller thread.
 * no locking is required since these types will be atomic on any machine with at least 32 bit architecture.
 */
static volatile uint16_t ADC_az_now;    // current az ADC value
static volatile uint16_t ADC_az_target; // target az ADC value
static volatile uint16_t ADC_el_now;    // current el ADC value
static volatile uint16_t ADC_el_target; // target el ADC value
static volatile int AZ_cmd_cw;          // set while commanding a cw az rotation
static volatile int AZ_cmd_ccw;         // set while commanding a ccw az rotation
static volatile int EL_cmd_up;          // set while commanding an upward el rotation
static volatile int EL_cmd_down;        // set while commanding a downward el rotation


/* these variables are used to detect motion. an active axis is considered stopped if N_STOPPED ADC
 * consecutive readings are the same.
 * N.B. we tried a exponential average but it was too susceptable to bogus readings
 */
static volatile uint16_t ADC_az_prev;   // previous ADC_az_now
static volatile uint16_t ADC_el_prev;   // previous ADC_el_now
static volatile int ADC_az_n_equal;     // number of consecutive ADC_az_now that are equal
static volatile int ADC_el_n_equal;     // number of consecutive ADC_el_now that are equal
#define N_EQUAL_STOPPED 4               // number of equal consecutive ADC readings considered stopped


/* handy derived states
 */
#define AZ_cmd_active()         (AZ_cmd_cw || AZ_cmd_ccw)
#define EL_cmd_active()         (EL_cmd_up || EL_cmd_down)
#define AZ_is_stuck()           (AZ_cmd_active() && ADC_az_n_equal >= N_EQUAL_STOPPED)
#define EL_is_stuck()           (EL_cmd_active() && ADC_el_n_equal >= N_EQUAL_STOPPED)
#define AZ_isat_ccw_lim()       (ADC_cal_ok && ADC_az_now < ADC_az_min + ADC_AZ_DEADBAND)
#define AZ_isat_cw_lim()        (ADC_cal_ok && ADC_az_max < ADC_az_now + ADC_AZ_DEADBAND)
#define EL_isat_down_lim()      (ADC_cal_ok && ADC_el_now < ADC_el_min + ADC_EL_DEADBAND)
#define EL_isat_up_lim()        (ADC_cal_ok && ADC_el_max < ADC_el_now + ADC_EL_DEADBAND)
#define AZ_is_wrapped()         (ADC_cal_ok && g5500_ADC_to_az(ADC_az_now) >= AZ_MOUNT_WRAP)


/* forward declation
 */
static void g5500_sim_mode_set (int type);




/***********************************************************************************************************
 *
 *
 * conversion functions between ADC and world coordinates
 *
 *
 ***********************************************************************************************************/

/* convert from azimuth, in degrees eastward from true north, to azimuth ADC count.
 * N.B. only valid when ADC_cal_ok
 */
static uint16_t g5500_az_to_ADC (float az)
{
    // beware no cal
    if (!ADC_cal_ok)
        return (0);

    // enforce valid range
    if (az < AZ_MOUNT_MIN)
        return (ADC_az_min);
    if (az > AZ_MOUNT_MAX)
        return (ADC_az_max);

    // linear conversion
    return (ADC_az_min + (az - AZ_MOUNT_MIN) * (ADC_az_max - ADC_az_min) / (AZ_MOUNT_MAX - AZ_MOUNT_MIN));
}


/* convert from elevation, in degrees up from the horizon, to elevation ADC count.
 * N.B. only valid when ADC_cal_ok
 */
static uint16_t g5500_el_to_ADC (float el)
{
    // beware no cal
    if (!ADC_cal_ok)
        return (0);

    // enforce valid range
    if (el < EL_MOUNT_MIN)
        return (ADC_el_min);
    if (el > el_mount_max)
        return (ADC_el_max);

    // linear conversion
    if (g5500_sim_mode == SIM_AZONLY)
        return(0);
    return (ADC_el_min + (el - EL_MOUNT_MIN) * (ADC_el_max - ADC_el_min) / (el_mount_max - EL_MOUNT_MIN));
}


/* convert the given azimuth ADC count to azimuth in degrees eastward from true north.
 * N.B. only valid when ADC_cal_ok
 */
static float g5500_ADC_to_az (uint16_t adc)
{
    // beware no cal
    if (!ADC_cal_ok)
        return (0);

    // enforce valid range
    if (adc < ADC_az_min)
        return (AZ_MOUNT_MIN);
    if (adc > ADC_az_max)
        return (AZ_MOUNT_MAX);

    return (AZ_MOUNT_MIN + (adc - ADC_az_min) * (AZ_MOUNT_MAX - AZ_MOUNT_MIN) / (ADC_az_max - ADC_az_min));
}


/* convert the given elevation ADC count to elevation in degrees up from horizon
 * N.B. only valid when ADC_cal_ok
 */
static float g5500_ADC_to_el (uint16_t adc)
{
    // beware no cal
    if (!ADC_cal_ok)
        return (0);

    // enforce valid range
    if (adc < ADC_el_min)
        return (EL_MOUNT_MIN);
    if (adc > ADC_el_max)
        return (el_mount_max);

    if (g5500_sim_mode == SIM_AZONLY)
        return(0);
    return (EL_MOUNT_MIN + (adc - ADC_el_min) * (el_mount_max - EL_MOUNT_MIN) / (ADC_el_max - ADC_el_min));
}




/***********************************************************************************************************
 *
 *
 * persistent calibration file storage
 *
 *
 ***********************************************************************************************************/


/* return full path to file containing calibration parameters,
 * or return NULL if can not be established.
 */
static const char* g5500_get_cal_filename()
{
    // set up path if first call
    static char *home, *path;
    if (!home)
        home = getenv ("HOME");
    if (!home)
        return (NULL);
    if (!path) {
        path = (char *) malloc (strlen(home) + strlen(g5500_cal_file_name) + 2);  // +1 for '/', +1 for '\0'
        if (!path)
            return (NULL);
        sprintf (path, "%s/%s", home, g5500_cal_file_name);
    }

    // return path
    return (path);
}

/* save the calibration constants to file.
 */
static void g5500_save_cal_file()
{
    const char *filename = g5500_get_cal_filename();
    FILE *fp = fopen (filename, "w");
    if (!fp)
        return;

    rig_debug(RIG_DEBUG_VERBOSE, "%s saving %s\n", __func__, filename);

    fprintf (fp, "ADC_az_min = %d\n", ADC_az_min);
    fprintf (fp, "ADC_az_max = %d\n", ADC_az_max);
    fprintf (fp, "ADC_el_min = %d\n", ADC_el_min);
    fprintf (fp, "ADC_el_max = %d\n", ADC_el_max);
    
    fclose(fp);

    
}


/* try to retrieve calibration constants from file.
 * if successful set ADC_cal_ok and return 0, otherwise return -1.
 */
static int g5500_read_cal_file()
{
    const char *filename;
    FILE *fp;
    char buf[1024];
    int tmp;
    int az_min_ok = 0;
    int az_max_ok = 0;
    int el_min_ok = 0;
    int el_max_ok = 0;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    // get full path to file
    filename = g5500_get_cal_filename();
    if (!filename)
        return (-1);

    // open
    fp = fopen (filename, "r");
    if (!fp)
        return (-1);

    rig_debug(RIG_DEBUG_VERBOSE, "%s found %s\n", __func__, filename);

    // read looking for each value
    while (fgets (buf, sizeof(buf), fp) != NULL) {
        if (sscanf (buf, "ADC_az_min = %d", &tmp) == 1) {
            ADC_az_min = (uint16_t) tmp;
            az_min_ok = 1;
        }
        if (sscanf (buf, "ADC_az_max = %d", &tmp) == 1) {
            ADC_az_max = (uint16_t) tmp;
            az_max_ok = 1;
        }
        if (sscanf (buf, "ADC_el_min = %d", &tmp) == 1) {
            ADC_el_min = (uint16_t) tmp;
            el_min_ok = 1;
        }
        if (sscanf (buf, "ADC_el_max = %d", &tmp) == 1) {
            ADC_el_max = (uint16_t) tmp;
            el_max_ok = 1;
        }
    }

    // finished with file
    fclose (fp);

    // require all values found
    if (!az_min_ok || !az_max_ok || !el_min_ok || !el_max_ok)
        return (-1);

    // sanity checks
    if (ADC_az_max < ADC_az_min+1000)
        return (-1);
    if (ADC_el_max < ADC_el_min+1000)
        return (-1);

    rig_debug(RIG_DEBUG_TRACE, "%s found AZ %u %u EL %u %u\n", __func__,
                ADC_az_min, ADC_az_max, ADC_el_min, ADC_el_max);

    // ok!
    ADC_cal_ok = 1;

    return (0);

}




/***********************************************************************************************************
 *
 *
 * control thread implementation
 *
 *
 ***********************************************************************************************************/




/* possible states of the controller thread
 */
typedef enum {
    CTS_STOP,                           // all relays off
    CTS_RUN,                            // seek ADC_az_target and ADC_el_target
    CTS_CAL_START,                      // start the calibration sequence
    CTS_CAL_SEEK_MINS,                  // moving to az and el min limits
    CTS_CAL_SEEK_MAXS,                  // moving to az and el max limits
    CTS_ERR_ADC,                        // ADC err
    CTS_ERR_NOPOWER,                    // no power
    CTS_ERR_STUCK,                      // not moving but should be
} G5500ControlThreadState;

static volatile G5500ControlThreadState g5500_thread_state = CTS_STOP;


/* control thread's polling and motion commence periods, microseconds
 */
#define THREAD_PERIOD           200000
#define MOTION_START_PERIOD     1000000


/* capture &rot->rot_state for use by control thread
 */
static struct rot_state *my_rot_state;



/* handy low-level rotation commands which also update the shadow state variables for use by main thread.
 * N.B. to be called only by g5500_control_thread()
 */
static void g5500_thread_az_stop()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_AZ_CW, PIN_IDLE);
        piGPIOsetHiLo (PIN_AZ_CCW, PIN_IDLE);
    }

    AZ_cmd_cw = 0;
    AZ_cmd_ccw = 0;
}
static void g5500_thread_el_stop()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_EL_UP, PIN_IDLE);
        piGPIOsetHiLo (PIN_EL_DOWN, PIN_IDLE);
    }

    EL_cmd_up = 0;
    EL_cmd_down = 0;
}
static void g5500_thread_rotate_cw()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_AZ_CCW, PIN_IDLE);
        piGPIOsetHiLo (PIN_AZ_CW, PIN_ACTIVE);
    }

    AZ_cmd_ccw = 0;
    AZ_cmd_cw = 1;
}
static void g5500_thread_rotate_ccw()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_AZ_CW, PIN_IDLE);
        piGPIOsetHiLo (PIN_AZ_CCW, PIN_ACTIVE);
    }

    AZ_cmd_cw = 0;
    AZ_cmd_ccw = 1;
}
static void g5500_thread_rotate_down()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_EL_UP, PIN_IDLE);
        piGPIOsetHiLo (PIN_EL_DOWN, PIN_ACTIVE);
    }

    EL_cmd_up = 0;
    EL_cmd_down = 1;
}
static void g5500_thread_rotate_up()
{
    if (g5500_sim_mode == SIM_OFF) {
        piGPIOsetHiLo (PIN_EL_DOWN, PIN_IDLE);
        piGPIOsetHiLo (PIN_EL_UP, PIN_ACTIVE);
    }

    EL_cmd_down = 0;
    EL_cmd_up = 1;
}

/* called by the control thread to capture the current mount status in the rot_state.has_status member.
 * this allows \dump_caps to show useful info about axis direction, limits etc.
 * N.B. we must use a pointer captured by g5500_direct_rot_init() which we must assume will never
 *      change. This hack would be unnecessary if \dump_caps was a proper callback like the other API calls.
 * N.B. to be called only by g5500_control_thread()
 */
static void g5500_thread_capture_state ()
{
    // at least check the pointer isn't null -- we can only assume it's still correct
    if (!my_rot_state)
        return;

    // zero flags then add as required
    my_rot_state->has_status = 0;

    if (AZ_cmd_cw)
        my_rot_state->has_status |= ROT_STATUS_MOVING_AZ | ROT_STATUS_MOVING_RIGHT;
    if (AZ_cmd_ccw)
        my_rot_state->has_status |= ROT_STATUS_MOVING_AZ | ROT_STATUS_MOVING_LEFT;
    if (EL_cmd_up)
        my_rot_state->has_status |= ROT_STATUS_MOVING_EL | ROT_STATUS_MOVING_UP;
    if (EL_cmd_down)
        my_rot_state->has_status |= ROT_STATUS_MOVING_EL | ROT_STATUS_MOVING_DOWN;
    if (AZ_isat_ccw_lim())
        my_rot_state->has_status |= ROT_STATUS_LIMIT_LEFT;
    if (AZ_isat_cw_lim())
        my_rot_state->has_status |= ROT_STATUS_LIMIT_RIGHT;
    if (EL_isat_down_lim())
        my_rot_state->has_status |= ROT_STATUS_LIMIT_DOWN;
    if (EL_isat_up_lim())
        my_rot_state->has_status |= ROT_STATUS_LIMIT_UP;
    if (AZ_is_wrapped())
        my_rot_state->has_status |= ROT_STATUS_OVERLAP_RIGHT;

    // use a switch statement so compiler will warn if later change the number of state values
    switch (g5500_thread_state) {
    case CTS_STOP:
    case CTS_RUN:
    case CTS_CAL_START:
    case CTS_CAL_SEEK_MINS:
    case CTS_CAL_SEEK_MAXS:
        my_rot_state->has_status |= ROT_STATUS_BUSY;
        break;
    case CTS_ERR_ADC:
    case CTS_ERR_NOPOWER:
    case CTS_ERR_STUCK:
        break;          // really wish there was an error flag for has_state
    }
}

/* called by thread to read the current position of each axis into ADC_az_now and ADC_el_now.
 * when simulating, just update at polling rate
 * N.B. to be called only by g5500_control_thread()
 */
static void g5500_thread_read_axis_positions()
{
    if (g5500_sim_mode == SIM_OFF) {

        // read real ADC -- important enough that we report errors regardless of rig_debug

        char ynot[1024];
        uint16_t adc;           // can't pass address of volatile

        // check power first
        if (readADC_SingleEnded (ADC_I2C_ADDR, ADC_CHANNEL_POK, &adc, ynot) < 0) {
            fprintf (stderr, "Power ADC read error: %s\n", ynot);
            g5500_thread_state = CTS_ERR_ADC;
            return;
        }
        if (adc < ADC_MIN_POK) {
            fprintf (stderr, "G5500 power off\n");
            g5500_thread_state = CTS_ERR_NOPOWER;
            return;
        }

        // read az and el
        if (readADC_SingleEnded (ADC_I2C_ADDR, ADC_CHANNEL_AZ, &adc, ynot) < 0) {
            fprintf (stderr, "AZ ADC read error: %s\n", ynot);
            g5500_thread_state = CTS_ERR_ADC;
            return;
        } else {
            ADC_az_now = adc;
        }

        if (readADC_SingleEnded (ADC_I2C_ADDR, ADC_CHANNEL_EL, &adc, ynot) < 0) {
            fprintf (stderr, "EL ADC read error: %s\n", ynot);
            g5500_thread_state = CTS_ERR_ADC;
            return;
        } else {
            ADC_el_now = adc;
        }

    } else {

        // pretend to move depending on the commanded motion state

        // ADC change per period
        #define AZ_SIM_ADC_PER_PRD      (AZ_SIM_SPEED*ADC_az_max/AZ_MOUNT_MAX*THREAD_PERIOD/1000000)
        #define EL_SIM_ADC_PER_PRD      (EL_SIM_SPEED*ADC_el_max/el_mount_max*THREAD_PERIOD/1000000)

        // update az

        if (AZ_cmd_cw) {
            ADC_az_now += AZ_SIM_ADC_PER_PRD;
            if (ADC_az_now > ADC_az_max)
                ADC_az_now = ADC_az_max;
        } else if (AZ_cmd_ccw) {
            if (ADC_az_now >= AZ_SIM_ADC_PER_PRD)
                ADC_az_now -= AZ_SIM_ADC_PER_PRD;
            else
                ADC_az_now = 0;
        }

        // update el, although might not be being used

        if (EL_cmd_up) {
            ADC_el_now += EL_SIM_ADC_PER_PRD;
            if (ADC_el_now > ADC_el_max)
                ADC_el_now = ADC_el_max;
        } else if (EL_cmd_down) {
            if (ADC_el_now >= EL_SIM_ADC_PER_PRD)
                ADC_el_now -= EL_SIM_ADC_PER_PRD;
            else
                ADC_el_now = 0;
        }
    }
}


/* this function is the separate control thread.
 * it loops forever doing whatever is required by g5500_thread_state.
  */
static void *g5500_control_thread (void *unused)
{
    (void) unused;

    // initially stop
    g5500_thread_state = CTS_STOP;
    g5500_thread_az_stop();
    g5500_thread_el_stop();

    // forever
    for(;;) {

        // read fresh positions
        g5500_thread_read_axis_positions();


        // update stopped detection metrics
        if (AZ_cmd_active() && ADC_az_now == ADC_az_prev) {
            // cap at N_EQUAL_STOPPED to avoid overflow if idle for long periods
            if (ADC_az_n_equal < N_EQUAL_STOPPED)
                ADC_az_n_equal++;
        } else {
            ADC_az_n_equal = 0;
        }
        if (EL_cmd_active() && ADC_el_now == ADC_el_prev) {
            // cap at N_EQUAL_STOPPED to avoid overflow if idle for long periods
            if (ADC_el_n_equal < N_EQUAL_STOPPED)
                ADC_el_n_equal++;
        } else {
            ADC_el_n_equal = 0;
        }


        // retain ADC values for next loop
        ADC_az_prev = ADC_az_now;
        ADC_el_prev = ADC_el_now;


        // publish status
        g5500_thread_capture_state();

        rig_debug(RIG_DEBUG_TRACE, "%s state %d AZ n= %d %4u -> %4u %6.1f %s  EL n= %d %4u -> %4u %6.1f %s\n",
                __func__, g5500_thread_state,
                ADC_az_n_equal, ADC_az_now, ADC_az_target, g5500_ADC_to_az (ADC_az_now),
                    AZ_cmd_cw ? " CW " : (AZ_cmd_ccw ? " CCW" : "STOP"),
                ADC_el_n_equal, ADC_el_now, ADC_el_target, g5500_ADC_to_el (ADC_el_now),
                    EL_cmd_up ? " UP " : (EL_cmd_down ? "DOWN" : "STOP"));


        // what we do next depends on our state
        switch (g5500_thread_state) {

        case CTS_STOP:

            // insure no commanded motions

            g5500_thread_az_stop();
            g5500_thread_el_stop();

            break;

        case CTS_RUN:

            // seek target

            if (AZ_is_stuck()) {

                // stop and report stuck az axis
                g5500_thread_az_stop();
                g5500_thread_state = CTS_ERR_STUCK;

            } else {

                // seek az target
                if (AZ_cmd_ccw) {
                    if (ADC_az_now <= ADC_az_target)
                        g5500_thread_az_stop();
                } else if (AZ_cmd_cw) {
                    if (ADC_az_now >= ADC_az_target)
                        g5500_thread_az_stop();
                } else if (ADC_az_now > ADC_az_target + ADC_AZ_DEADBAND) {
                    g5500_thread_rotate_ccw();
                } else if (ADC_az_now + ADC_AZ_DEADBAND < ADC_az_target) {
                    g5500_thread_rotate_cw();
                } else {
                    g5500_thread_az_stop();
                }

            }

            if (EL_is_stuck()) {

                // stop and report stuck el axis
                g5500_thread_el_stop();
                g5500_thread_state = CTS_ERR_STUCK;

            } else {

                // seek el target
                if (EL_cmd_down) {
                    if (ADC_el_now <= ADC_el_target)
                        g5500_thread_el_stop();
                } else if (EL_cmd_up) {
                    if (ADC_el_now >= ADC_el_target)
                        g5500_thread_el_stop();
                } else if (ADC_el_now > ADC_el_target + ADC_EL_DEADBAND) {
                    g5500_thread_rotate_down();
                } else if (ADC_el_now + ADC_EL_DEADBAND < ADC_el_target) {
                    g5500_thread_rotate_up();
                } else {
                    g5500_thread_el_stop();
                }

            }

            break;

        case CTS_CAL_START:

            // start the calibration sequence by moving to axis minima

            rig_debug(RIG_DEBUG_VERBOSE, "%s seeking mins\n", __func__);

            g5500_thread_rotate_ccw();
            g5500_thread_rotate_down();
            g5500_thread_state = CTS_CAL_SEEK_MINS;

            // give axes time to start moving to avoid false detection of finding min
            usleep (MOTION_START_PERIOD);

            break;

        case CTS_CAL_SEEK_MINS:

            // found axis minima when both stop moving, presumably because of limits

            rig_debug(RIG_DEBUG_TRACE, "%s seeking mins ADC az %u el %u\n", __func__, ADC_az_now, ADC_el_now);

            if (AZ_is_stuck() && EL_is_stuck()) {

                // record ADC at minima
                ADC_az_min = ADC_az_now;
                ADC_el_min = ADC_el_now;

                // proceed to both maxima
                g5500_thread_rotate_cw();
                g5500_thread_rotate_up();
                g5500_thread_state = CTS_CAL_SEEK_MAXS;

                rig_debug(RIG_DEBUG_VERBOSE, "%s seeking maxs\n", __func__);

                // give axes time to start moving to avoid false detection of finding max
                usleep (MOTION_START_PERIOD);
            }

            break;

        case CTS_CAL_SEEK_MAXS:

            // found axis maxima when both stop moving, presumably because of limits

            rig_debug(RIG_DEBUG_TRACE, "%s seeking maxs ADC az %u el %u\n", __func__, ADC_az_now, ADC_el_now);

            if (AZ_is_stuck() && EL_is_stuck()) {

                // record ADC at maxima
                ADC_az_max = ADC_az_now;
                ADC_el_max = ADC_el_now;

                // save new config values
                g5500_save_cal_file();

                // stop
                g5500_thread_az_stop();
                g5500_thread_el_stop();
                g5500_thread_state = CTS_STOP;
            }

            break;

        case CTS_ERR_ADC:
        case CTS_ERR_NOPOWER:
        case CTS_ERR_STUCK:

            // error state, insure no commanded motions

            g5500_thread_az_stop();
            g5500_thread_el_stop();

            break;

        }

        // poll delay
        usleep (THREAD_PERIOD);
    }

    // lint
    return (NULL);

}




/***********************************************************************************************************
 *
 *
 * hamlib API helper functions
 *
 *
 ***********************************************************************************************************/


/* called by the main thread to create and start the g5500 monitor/control thread running.
 * return 0 if ok else -1
 */
static int g5500_thread_create()
{
    pthread_t tid;
    return (pthread_create (&tid, NULL, g5500_control_thread, NULL) == 0 ? 0 : -1);
}


/* called by the main thread to tell controller thread to begin the calibration procedure if not already.
 */
static void g5500_tell_thread_start_calibration()
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    // use a switch so compiler will warn if later change the number of state values
    switch (g5500_thread_state) {

    case CTS_STOP:
    case CTS_RUN:
        // ok to begin
        g5500_thread_state = CTS_CAL_START;

    case CTS_CAL_START:
    case CTS_CAL_SEEK_MINS:
    case CTS_CAL_SEEK_MAXS:
        // just let calibration continue
        break;

    case CTS_ERR_ADC:
    case CTS_ERR_NOPOWER:
    case CTS_ERR_STUCK:
        // don't try anything
        break;
    }
}

/* called by the main thread to tell controller thread to stop all motion
 */
static void g5500_tell_thread_all_stop()
{
    g5500_thread_state = CTS_STOP;
}


/* check for any g5500_thread_state errors.
 * return different RIG_ values to at least differentiate them, albeit without any true meaning.
 * return G5500_RIG_OK if all ok.
 */
static int g5500_check_thread_error()
{
    // check thread states, use a switch to get a compiler warning in case the CTS_ enum ever changes
    switch (g5500_thread_state) {
    case CTS_STOP:
    case CTS_RUN:
    case CTS_CAL_START:
    case CTS_CAL_SEEK_MINS:
    case CTS_CAL_SEEK_MAXS:
        // these are fine
        break;
    case CTS_ERR_ADC:
        return G5500_RIG_ERR_ADC;
    case CTS_ERR_NOPOWER:
        return G5500_RIG_ERR_NOPOWER;
    case CTS_ERR_STUCK:
        return G5500_RIG_ERR_STUCK;
        break;
    }

    // ok!
    return G5500_RIG_OK;
}

/* called by hamlib API functions that require calibration.
 */
static int g5500_cal_ready()
{
    // check for thread errors
    int err = g5500_check_thread_error();
    if (err != G5500_RIG_OK) {
        // restart state after reporting once
        g5500_tell_thread_all_stop();
        return err;
    }

    // cal ok if already set or can be set from a valid file
    if (ADC_cal_ok || g5500_read_cal_file() == 0) {
        return G5500_RIG_OK;
    }

    // not calibrated and prior values are not available so start the calibration procedure
    g5500_tell_thread_start_calibration();

    // busy
    return G5500_RIG_CALIBRATING;
}



/***********************************************************************************************************
 *
 *
 * hamlib API functions
 *
 *
 ***********************************************************************************************************/




/* 
 * Init: Called exactly one time before any other API calls to initialize the driver.
 */
static int g5500_direct_rot_init (ROT *rot)
{
    // hack to capture the mutable state for use by thread
    if (!rot)
    {
        return G5500_RIG_ERR_BADARGS;
    }
    my_rot_state = &rot->state;

    #if defined(ISA_PI)

        rig_debug(RIG_DEBUG_VERBOSE, "RPi %s called\n", __func__);

        // prepare GPIO and I2C subsystems
        char ynot[1024];
        if (piGPIOInit(ynot) < 0) {
            rig_debug(RIG_DEBUG_ERR, "GPIO error: %s\n", ynot);
            return G5500_RIG_ERR_GPIO;
        }
        piGPIOsetAsOutput (PIN_AZ_CW);
        piGPIOsetAsOutput (PIN_AZ_CCW);
        piGPIOsetAsOutput (PIN_EL_UP);
        piGPIOsetAsOutput (PIN_EL_DOWN);
        if (piI2CInit(ynot) < 0) {
            rig_debug(RIG_DEBUG_ERR, "I2C error: %s\n", ynot);
            return G5500_RIG_ERR_ADC;
        }

    #else

        rig_debug(RIG_DEBUG_VERBOSE, "!RPi %s called\n", __func__);

        // engage simulator when not a RPi
        g5500_sim_mode_set (SIM_EL180);

    #endif

    // start control thread
    if (g5500_thread_create() < 0)
        return G5500_RIG_ERR_INTERNAL;

    // ready
    return G5500_RIG_OK;
}


/*
 * Set position
 */

static int g5500_direct_set_position (ROT *rot, azimuth_t azimuth, elevation_t elevation)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s (%g, %g) called\n", __func__, azimuth, elevation);

    // not used
    (void) rot;

    // require or start cal
    int err = g5500_cal_ready();
    if (err != G5500_RIG_OK)
    {
        return err;
    }

    // sanity check args
    if (azimuth < AZ_MOUNT_MIN || azimuth > AZ_MOUNT_MAX)
    {
        return G5500_RIG_ERR_BADARGS;
    }
    if (elevation < EL_MOUNT_MIN || elevation > el_mount_max)
    {
        return G5500_RIG_ERR_BADARGS;
    }


    // update control thread targets and go
    ADC_az_target = g5500_az_to_ADC (azimuth);
    ADC_el_target = g5500_el_to_ADC (elevation);
    g5500_thread_state = CTS_RUN;

    return G5500_RIG_OK;
}

/*
 * Get position
 */
static int g5500_direct_get_position (ROT *rot, azimuth_t *azimuth, elevation_t *elevation)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    // not used
    (void) rot;

    // check for pending thread errors
    int err = g5500_check_thread_error();
    if (err != G5500_RIG_OK)
    {
        return err;
    }

    // require or start cal
    err = g5500_cal_ready();
    if (err != G5500_RIG_OK)
    {
        return err;
    }

    // return current values
    *azimuth = g5500_ADC_to_az (ADC_az_now);
    *elevation = g5500_ADC_to_el (ADC_el_now);

    rig_debug(RIG_DEBUG_VERBOSE, "%s returns %g, %g\n", __func__, *azimuth, *elevation);

    return G5500_RIG_OK;
}



/* 
 * Get Info
 *
 * not much info, just the name??
 *
 */
static const char *g5500_direct_get_info(ROT *rot)
{
    // not used
    (void) rot;

    return ("Yaesu G5500 on RPi");
}


/* 
 * Set a g5500_direct configuration parameter
 */
static int g5500_direct_set_conf (ROT *rot, token_t token, const char *val)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s (%d,%s)\n", __func__, (int)token, val);

    // not used
    (void) rot;

    int tmp;

    switch (token) {

    case TOK_SIMULATOR:
        // set desired simulator mode
        tmp = atoi (val);
        g5500_sim_mode_set (tmp);
        break;

    default:
        return G5500_RIG_ERR_BADARGS;
    }

    return G5500_RIG_OK;
}


/* 
 * get a g5500_direct configuration parameter
 */
static int g5500_direct_get_conf (ROT *rot, token_t token, char *val)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s(%ld) called\n", __func__, token);

    // not used
    (void) rot;

    switch (token) {

    case TOK_SIMULATOR:
        sprintf (val, "%d", (int)g5500_sim_mode);
        break;

    default:
        return G5500_RIG_ERR_BADARGS;
    }

    rig_debug(RIG_DEBUG_VERBOSE, "%s() returns %s\n", __func__, val);

    return G5500_RIG_OK;
}


/* 
 * Move
 */
static int g5500_direct_move (ROT *rot, int direction, int speed)
{
    // not used
    (void) rot;

    // we have no means to control speed.
    (void) speed;

    rig_debug(RIG_DEBUG_VERBOSE, "%s (%d, %d) called\n", __func__, direction, speed);

    // get ADC calibration values or start the cal procedure
    int err = g5500_cal_ready();
    if (err != G5500_RIG_OK)
    {
        return err;
    }

    switch (direction)
    {
    case ROT_MOVE_UP:       /* Elevation increase */
        ADC_el_target = ADC_el_max;
        g5500_thread_state = CTS_RUN;
        break;

    case ROT_MOVE_DOWN:     /* Elevation decrease */
        ADC_el_target = ADC_el_min;
        g5500_thread_state = CTS_RUN;
        break;

    case ROT_MOVE_LEFT:     /* Azimuth decrease */
        ADC_az_target = ADC_az_min;
        g5500_thread_state = CTS_RUN;
        break;

    case ROT_MOVE_RIGHT:    /* Azimuth increase */
        ADC_az_target = ADC_az_max;
        g5500_thread_state = CTS_RUN;
        break;

    default:
        rig_debug(RIG_DEBUG_ERR, "%s: Invalid direction value! (%d)\n", __func__, direction);
        return G5500_RIG_ERR_BADARGS;
    }

    return G5500_RIG_OK;
}



/* 
 * Park
 */
static int g5500_direct_park (ROT *rot)
{
    // not used
    (void) rot;

    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    // get ADC calibration values or start the cal procedure
    int err = g5500_cal_ready();
    if (err != G5500_RIG_OK)
    {
        return err;
    }

    ADC_az_target = g5500_az_to_ADC (AZ_MOUNT_PARK);
    ADC_el_target = g5500_el_to_ADC (EL_MOUNT_PARK);
    g5500_thread_state = CTS_RUN;

    return G5500_RIG_OK;
}



/*
 * Stop rotation
 */

static int g5500_direct_stop(ROT *rot)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    // not used
    (void) rot;

    // inform thread
    g5500_tell_thread_all_stop();

    return G5500_RIG_OK;
}


/*
 * list of local configuration parameters
 */
static const struct confparams g5500_direct_conf_params[] =
{
    {
        TOK_SIMULATOR, "simulator", "Simulate mount", "Simulate mount",
        NULL, RIG_CONF_NUMERIC, { .n.min = 0, .n.max = 3, .n.step = 1 }
    },
    { RIG_CONF_END, NULL, }
};


/*
 * set up hamlib G5500 rotor capabilities
 *
 */

static struct rot_caps g5500_direct_caps =
{
    ROT_MODEL(ROT_MODEL_G5500_DIRECT),
    .model_name =         "G5500",
    .mfg_name =           "Yaesu",
    .version =            "20220228.0",
    .copyright =          "LGPL",
    .status =             RIG_STATUS_BETA,
    .rot_type =           ROT_TYPE_OTHER,
    .port_type =          RIG_PORT_NONE,

    .min_az =             AZ_MOUNT_MIN,
    .max_az =             AZ_MOUNT_MAX,
    .min_el =             EL_MOUNT_MIN,
    .max_el =             EL_MOUNT_MAX,         // can be changed by g5500_sim_mode_set

    .cfgparams =          g5500_direct_conf_params,
    .priv =               NULL,

    .rot_init =           g5500_direct_rot_init,
    .set_conf =           g5500_direct_set_conf,
    .get_conf =           g5500_direct_get_conf,
    .set_position =       g5500_direct_set_position,
    .get_position =       g5500_direct_get_position,
    .stop         =       g5500_direct_stop,
    .park         =       g5500_direct_park,
    .move         =       g5500_direct_move,
    .get_info     =       g5500_direct_get_info,
};

/*
 * register this backend
 */

DECLARE_INITROT_BACKEND(g5500_direct)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

    rot_register(&g5500_direct_caps);

    return G5500_RIG_OK;
}


/* set simulator mode.
 * down here in order to access struct g5500_direct_caps
 */
static void g5500_sim_mode_set (int type)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s(%d)\n", __func__, type);

    switch ((SimType)type) {

    default:    // all other values enable real operation
    case SIM_OFF: 
        g5500_sim_mode = SIM_OFF;
        el_mount_max = g5500_direct_caps.max_el = EL_MOUNT_MAX;
        ADC_cal_ok = 0;         // force read of cal file
        break;

    case SIM_AZONLY: 
        g5500_sim_mode = SIM_AZONLY;
        el_mount_max = 1;       // avoid /0
        g5500_direct_caps.max_el = 0;
        ADC_az_min = 0;
        ADC_az_max = AZ_SIM_MAX_ADC;
        ADC_el_min = 0;
        ADC_el_max = EL_SIM_MAX_ADC;
        ADC_cal_ok = 1;
        break;

    case SIM_EL90: 
        g5500_sim_mode = SIM_EL90;
        el_mount_max = g5500_direct_caps.max_el = 90;
        ADC_az_min = 0;
        ADC_az_max = AZ_SIM_MAX_ADC;
        ADC_el_min = 0;
        ADC_el_max = EL_SIM_MAX_ADC/2;
        ADC_cal_ok = 1;
        break;

    case SIM_EL180: 
        g5500_sim_mode = SIM_EL180;
        el_mount_max = g5500_direct_caps.max_el = 180;
        ADC_az_min = 0;
        ADC_az_max = AZ_SIM_MAX_ADC;
        ADC_el_min = 0;
        ADC_el_max = EL_SIM_MAX_ADC;
        ADC_cal_ok = 1;
        break;
    }

    // common to all
    g5500_thread_state = CTS_STOP;
    AZ_cmd_cw = 0;
    AZ_cmd_ccw = 0;
    EL_cmd_up = 0;
    EL_cmd_down = 0;
    ADC_az_now = 0;
    ADC_az_target = 0;
    ADC_az_n_equal = 0;
    ADC_el_now = 0;
    ADC_el_target = 0;
    ADC_el_n_equal = 0;
}
