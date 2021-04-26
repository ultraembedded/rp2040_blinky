/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "gpio.h"
#include "hardware/structs/iobank0.h"

// Get the raw value from the pin, bypassing any muxing or overrides.
int gpio_get_pad(uint gpio) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_set_bits(&padsbank0_hw->io[gpio], PADS_BANK0_GPIO0_IE_BITS);
    return (iobank0_hw->io[gpio].status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS)
            >> IO_BANK0_GPIO0_STATUS_INFROMPAD_LSB;
}

/// \tag::gpio_set_function[]
// Select function for this GPIO, and ensure input/output are enabled at the pad.
// This also clears the input/output/irq override bits.
void gpio_set_function(uint gpio, enum gpio_function fn) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    invalid_params_if(GPIO, ((uint32_t)fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB) & ~IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS);
    // Set input enable on, output disable off
    hw_write_masked(&padsbank0_hw->io[gpio],
                   PADS_BANK0_GPIO0_IE_BITS,
                   PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS
    );
    // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
    // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
    iobank0_hw->io[gpio].ctrl = fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}
/// \end::gpio_set_function[]

enum gpio_function gpio_get_function(uint gpio) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    return (enum gpio_function) ((iobank0_hw->io[gpio].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
}

// Note that, on RP2040, setting both pulls enables a "bus keep" function,
// i.e. weak pull to whatever is current high/low state of GPIO.
void gpio_set_pulls(uint gpio, bool up, bool down) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_write_masked(
            &padsbank0_hw->io[gpio],
            (bool_to_bit(up) << PADS_BANK0_GPIO0_PUE_LSB) | (bool_to_bit(down) << PADS_BANK0_GPIO0_PDE_LSB),
            PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS
    );
}

// Direct overrides for pad controls
void gpio_set_inover(uint gpio, uint value) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_write_masked(&iobank0_hw->io[gpio].ctrl,
                   value << IO_BANK0_GPIO0_CTRL_INOVER_LSB,
                   IO_BANK0_GPIO0_CTRL_INOVER_BITS
    );
}

void gpio_set_outover(uint gpio, uint value) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_write_masked(&iobank0_hw->io[gpio].ctrl,
                   value << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB,
                   IO_BANK0_GPIO0_CTRL_OUTOVER_BITS
    );
}

void gpio_set_oeover(uint gpio, uint value) {
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_write_masked(&iobank0_hw->io[gpio].ctrl,
                   value << IO_BANK0_GPIO0_CTRL_OEOVER_LSB,
                   IO_BANK0_GPIO0_CTRL_OEOVER_BITS
    );
}

void gpio_set_input_enabled(uint gpio, bool enabled) {
    if (enabled)
        hw_set_bits(&padsbank0_hw->io[gpio], PADS_BANK0_GPIO0_IE_BITS);
    else
        hw_clear_bits(&padsbank0_hw->io[gpio], PADS_BANK0_GPIO0_IE_BITS);
}

void gpio_init(uint gpio) {
    sio_hw->gpio_oe_clr = 1ul << gpio;
    sio_hw->gpio_clr = 1ul << gpio;
    gpio_set_function(gpio, GPIO_FUNC_SIO);
}

void gpio_init_mask(uint gpio_mask) {
    for(uint i=0;i<32;i++) {
        if (gpio_mask & 1) {
            gpio_init(i);
        }
        gpio_mask >>= 1;
    }
}

