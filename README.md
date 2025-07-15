# PicoCalc "southbridge" custom BIOS

This is my personnal rewrite of the [original](https://github.com/clockworkpi/PicoCalc/tree/master/Code/picocalc_keyboard)
PicoCalc STM32F103R8T6 firmware.

## Features differences
The main differences with the original firmware are the followings:

- drastic reduction in the STM32's electricity consumption when running (~3.5 mA),
- clean up (by removing stm32duino dependencies, use STM32HAL instead, maybe I'll switch to libopencm3 someday...) to reduce binary size (~25 KB) and allow more features to be implemented,
- added configuration saving solution (using internal flash, including backlight option),
- new I2C registers memory allocation and structure for cleaner evolution in feature (pico I2C driver updated to handle this version along with official)(WIP),
- interrupt event output to pico board can be configured during runtime (for keyboard event or RTC alarm),
- rewriten or added some debug UART interface message (only when compiled in DEBUG release type),
- internal RTC access through dedicated I2C registers,
- auto wake-up using RTC (WIP),
- lighten AXP2101 PMIC driver (based on X-PowersLib).

## Compile
This source code can be compiled using ARM gcc toolchain (using v13) in path and using make program.

CAUTION: By default, I2C registers use a new, exploded structure that is more flexible! 
But this makes them incompatible with official firmware. (More details to come about I2C registers structure in the wiki...)

If you plan using pico official firmware (PicoMite, etc.), you should set I2C_REGS_COMPAT = 1 in the Makefile.

## TODO
- Registers memory structure/allocation rewrite
- IRQ to Pico management (register exist but does nothing)
- Add a Pico test program for registers/features implemented
- add few wiki page to detail the I2C protocol, added features, etc.

## Important notes
The current implementation of this firmware is subject to change until the v1 release.

Some features can be unstable or buggy and are marked as pre-release. A test program will be developped soon to provide some regression testing.

The permanent settings (EEPROM) save can be broken between 0.x version, it is recommanded to make a full flash erase before updating, as 
EEPROM configuration survives update.

## Credits
- STM32-HAL: [link](https://github.com/STMicroelectronics/stm32f1xx-hal-driver)
- STM32-CMSIS: [link](https://github.com/STMicroelectronics/cmsis-device-f1)
- X-PowersLib: [link](https://github.com/lewisxhe/XPowersLib)
- Emulated EEPROM: [link](https://github.com/ScorpionX5/STM32F1XX-EEPROM-Emulation-Library/tree/master/V2.0)
