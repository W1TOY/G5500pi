/* include file for use when building this stand-alone, ie, outside the hamlib development environment.
 * defines the bare minimum we need from rotator.h, rig.h and register.h
 */

#ifndef _SA_G500_H
#define _SA_G500_H

#include <stdio.h>
#include <stdarg.h>

enum rig_debug_level_e {
    RIG_DEBUG_NONE = 0,
    RIG_DEBUG_BUG,
    RIG_DEBUG_ERR,
    RIG_DEBUG_WARN,
    RIG_DEBUG_VERBOSE,
    RIG_DEBUG_TRACE,
};


extern int sendWebPage (FILE *fp);
extern void rig_debug (int level, const char *fmt, ...);

typedef enum {
    ROT_STATUS_NONE =              0,
    ROT_STATUS_BUSY =              (1 << 0),
    ROT_STATUS_MOVING =            (1 << 1),
    ROT_STATUS_MOVING_AZ =         (1 << 2),
    ROT_STATUS_MOVING_LEFT =       (1 << 3),
    ROT_STATUS_MOVING_RIGHT =      (1 << 4),
    ROT_STATUS_MOVING_EL =         (1 << 5),
    ROT_STATUS_MOVING_UP =         (1 << 6),
    ROT_STATUS_MOVING_DOWN =       (1 << 7),
    ROT_STATUS_LIMIT_UP =          (1 << 8),
    ROT_STATUS_LIMIT_DOWN =        (1 << 9),
    ROT_STATUS_LIMIT_LEFT =        (1 << 10),
    ROT_STATUS_LIMIT_RIGHT =       (1 << 11),
    ROT_STATUS_OVERLAP_UP =        (1 << 12),
    ROT_STATUS_OVERLAP_DOWN =      (1 << 13),
    ROT_STATUS_OVERLAP_LEFT =      (1 << 14),
    ROT_STATUS_OVERLAP_RIGHT =     (1 << 16),
} rot_status_t;

enum rig_errcode_e {
    RIG_OK = 0,
    RIG_EINVAL,
    RIG_ECONF,
    RIG_ENOMEM,
    RIG_ENIMPL,

    RIG_ETIMEOUT,
    RIG_EIO,
    RIG_EINTERNAL,
    RIG_EPROTO,
    RIG_ERJCTED,

    RIG_ETRUNC,
    RIG_ENAVAIL,
    RIG_ENTARGET,
    RIG_BUSERROR,
    RIG_BUSBUSY,

    RIG_EARG,
    RIG_EVFO,
    RIG_EDOM,
};

enum {
    ROT_MOVE_UP = 2,
    ROT_MOVE_DOWN = 4,
    ROT_MOVE_LEFT = 8,
    ROT_MOVE_RIGHT = 16,
};

struct rot_state {
    int has_status;
};

typedef struct {
    struct rot_state state;
} ROT;

typedef float azimuth_t;
typedef float elevation_t;
typedef int token_t;

/* strongly inspired from soundmodem. Thanks Thomas! */
enum rig_conf_e {
    RIG_CONF_STRING,        /*!<    String type */
    RIG_CONF_COMBO,         /*!<    Combo type */
    RIG_CONF_NUMERIC,       /*!<    Numeric type integer or real */
    RIG_CONF_CHECKBUTTON,   /*!<    on/off type */
    RIG_CONF_BUTTON,        /*!<    Button type */
    RIG_CONF_BINARY         /*!<    Binary buffer type */
};
#define RIG_CONF_END 0


/**
 * \brief Configuration parameter structure.
 */
struct confparams {
    token_t token;          /*!< Conf param token ID */
    const char *name;       /*!< Param name, no spaces allowed */
    const char *label;      /*!< Human readable label */
    const char *tooltip;    /*!< Hint on the parameter */
    const char *dflt;       /*!< Default value */
    enum rig_conf_e type;   /*!< Type of the parameter */
    union {                 /*!< */
        struct {            /*!< */
            float min;      /*!< Minimum value */
            float max;      /*!< Maximum value */
            float step;     /*!< Step */
        } n;                /*!< Numeric type */
    } u;                    /*!< Type union */
};

#define RIG_STATUS_BETA 	0
#define ROT_TYPE_OTHER  	0
#define RIG_PORT_NONE   	0
#define ROT_MODEL_G5500_DIRECT  0
#define ROT_MODEL(x)    	.rot_model = x

struct rot_caps {
    const char *model_name;
    const char *mfg_name;
    const char *version;
    const char *copyright;
    int rot_model;
    int status;
    int rot_type;
    int port_type;

    float min_az, max_az, min_el, max_el;

    const struct confparams *cfgparams;
    char *priv;

    int (*rot_init) (ROT*);
    int (*set_conf) (ROT*, token_t, const char*);
    int (*get_conf) (ROT*, token_t, char*);
    int (*set_position) (ROT*, azimuth_t, elevation_t);
    int (*get_position) (ROT*, azimuth_t*, elevation_t*);
    int (*move) (ROT*, int, int);
    int (*stop) (ROT*);
    int (*park) (ROT*);
    const char *(*get_info) (ROT*);
};

#define DECLARE_INITROT_BACKEND(x)      int DECLARE_INITROT_BACKEND_dummy()
extern int DECLARE_INITROT_BACKEND_dummy();
extern void rot_register (struct rot_caps *);

#endif // _SA_G500_H
