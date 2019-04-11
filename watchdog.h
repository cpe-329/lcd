#ifndef __MY_WATCHDOG_H__
#define __MY_WATCHDOG_H__

// Macro for disabling watchdog
#define DISABLE_WATCHDOG  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

#endif
