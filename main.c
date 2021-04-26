#include "timer.h"
#include "gpio.h"

#define PICO_DEFAULT_LED_PIN 25

//-----------------------------------------------------------------
// Main: Main loop
//-----------------------------------------------------------------
int main(void)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(LED_PIN, 1);
        timer_sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        timer_sleep_ms(1000);
    }

    return 0;
}