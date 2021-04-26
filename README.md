# rp2040_blinky
Simple blinky example for the RP2040 that does not require cmake.

### Usage
```
# Acquire official RP2040 SDK
git clone https://github.com/raspberrypi/pico-sdk.git

# Set PICO_SDK_PATH to wherever you cloned the PICO SDK to
export PICO_SDK_PATH=/path/to/pico-SDK

# Build the blinky example
cd rp2040_blinky
make

# Copy test.uf2 to the RP2040 in the usual manner
```
