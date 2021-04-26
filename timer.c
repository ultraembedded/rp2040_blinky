#include "pico.h"
#include "hardware/structs/timer.h"

//-----------------------------------------------------------------
// timer_now: Time in uS
//-----------------------------------------------------------------
uint64_t timer_now(void)
{
    uint32_t hi1 = timer_hw->timerawh;
    uint32_t hi2;
    uint32_t lo;
    do
    {
        lo  = timer_hw->timerawl;
        hi2 = timer_hw->timerawh;
    }
    while (hi2 != hi1);

    return ((uint64_t) hi2 << 32u) | lo;
}
//-----------------------------------------------------------------
// timer_sleep_us: Busy wait for a number of uS
//-----------------------------------------------------------------
void timer_sleep_us(uint32_t us)
{
    uint64_t te = timer_now() + us;
    while (timer_now() < te)
        ;
}
//-----------------------------------------------------------------
// timer_sleep_ms: Busy wait for a number of ms
//-----------------------------------------------------------------
void timer_sleep_ms(uint32_t ms)
{
    uint64_t te = timer_now() + (ms * 1000);
    while (timer_now() < te)
        ;
}
