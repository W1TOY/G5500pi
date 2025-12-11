#ifndef _ISA_PI_H
#define _ISA_PI_H

// define ISA_PI if this system appears to be a Raspberry Pi running linux
#if (defined(__arm__) || defined(__aarch64__)) && defined(__linux__)
    #if defined(__has_include)
        #if __has_include(<bcm_host.h>)
            #define ISA_PI
        #endif
    #endif
#endif

#endif  // _ISA_PI_H
