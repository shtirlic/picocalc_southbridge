# PicoCalc "southbridge" custom BIOS

This is my personnal rewrite of the [original](https://github.com/clockworkpi/PicoCalc/tree/master/Code/picocalc_keyboard)
PicoCalc STM32F103R8T6 firmware.

## Features differences
The main differences with the original firmware are the followings:

- drastic reduction in the STM32's electricity consumption when running (~3.5 mA),
- clean up (in progress) to reduce binary size (~25 KB) and allow more features to be implemented,
- removed stm32duino dependencies (use STM32HAL instead, maybe I'll switch to libopencm3 someday...),
- added configuration saving solution (using internal flash, including backlight option),
- rewriten or added some debug UART interface message,
- lighten AXP2101 PMIC driver.

## Compile
This source code can be compiled using ARM gcc toolchain (using v13) in path and using make program.

## Credits
- STM32-HAL: [link](https://github.com/STMicroelectronics/stm32f1xx-hal-driver)
- STM32-CMSIS: [link](https://github.com/STMicroelectronics/cmsis-device-f1)
- X-PowersLib: [link](https://github.com/lewisxhe/XPowersLib)
- Emulated EEPROM: [link](https://github.com/ScorpionX5/STM32F1XX-EEPROM-Emulation-Library/tree/master/V2.0)
