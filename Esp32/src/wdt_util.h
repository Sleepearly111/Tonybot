#ifndef WDT_UTIL_H
#define WDT_UTIL_H

#include <Arduino.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// WDT-safe delay — feeds TG1WDT every 10ms during wait.
// Use instead of delay() to prevent TG1WDT_SYS_RESET in long init sequences.
inline void safeDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        delay(10);
        TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
        TIMERG1.wdt_feed = 1;
        TIMERG1.wdt_wprotect = 0;
    }
}

#endif
