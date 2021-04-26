#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

//-----------------------------------------------------------------
// Prototypes:
//-----------------------------------------------------------------
// Time in uS
uint64_t timer_now(void);

// Busy wait for a number of uS
void timer_sleep_us(uint32_t us);

// Busy wait for a number of ms
void timer_sleep_ms(uint32_t ms);

#endif
