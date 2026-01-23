# PicoCalc "southbridge" custom BIOS

This is my personnal rewrite of the [original](https://github.com/clockworkpi/PicoCalc/tree/master/Code/picocalc_keyboard)
PicoCalc STM32F103R8T6 firmware.

Some detailed [wiki pages](https://git.jcsmith.fr/jackcartersmith/picocalc_BIOS/wiki) are available.

The primary objectives of this firmware are to be more efficient both functionally and electrically.
This include:

- drastic reduction in the STM32's electricity consumption when running (~3.5 mA),
- switch backend lib stm32duino to STM32HAL. It's help reducing binary size (~25 KB) and allow more features to be implemented that way,
- added configuration saving solution (using internal flash, including backlight option),
- internal RTC access through dedicated I2C registers,
- main MCU (pico) reset using power button (Shift + short press on PWR button),
- lighten components drivers,
- auto wake-up using RTC (WIP)...

## Tools version

- ARM GNU GCC: 14.3-rel1_arm-none-eabi (but STM32 one can do the job too)

## Compile
This source code can be compiled using ARM/STM32 gcc toolchain in path and using make program.

```
git clone --recurse-submodules https://git.jcsmith.fr/jackcartersmith/picocalc_BIOS.git
cd picocalc_BIOS
make -j
```

## Programming

1. Unplug the picocalc and disconnect the batteries.
2. Open the picocalc to access the DIP switch (SW701), put the 1 pin to ON.
3. Connect the USB from the motherboard to the computer and press the power button.
4. Open STM32CubeProgrammer, look for the UART access and connect.
5. Use the compiled .bin/.elf firmware in the build folder and flash it on the STM32.
6. Put the 1 pin of SW701 to OFF.
7. Power reset everything like step 1.

## TODO
- Review the I2C slave... Can be better.

## Credits
- STM32-HAL: [link](https://github.com/STMicroelectronics/stm32f1xx-hal-driver)
- STM32-CMSIS: [link](https://github.com/STMicroelectronics/cmsis-device-f1)
- X-PowersLib: [link](https://github.com/lewisxhe/XPowersLib)
- Emulated EEPROM: [link](https://github.com/ScorpionX5/STM32F1XX-EEPROM-Emulation-Library/tree/master/V2.0)
