# PicoCalc "southbridge" custom BIOS

This is my personnal rewrite of the [original](https://github.com/clockworkpi/PicoCalc/tree/master/Code/picocalc_keyboard)
PicoCalc STM32F103R8T6 firmware.

The main differences with the original are followings:

- drastic reduction in the STM32's electricity consumption when running (~3.5 mA),
- removed stm32duino dependencies (use STM32HAL instead, maybe I'll switch to libopencm3 someday...),
- added configuration saving solution (using internal flash, including backlight option),
- rewriten or added some debug UART interface message,
- lighten AXP2101 PMIC driver.

This source code can be compiled using ARM gcc toolchain (using v13) and make program.
