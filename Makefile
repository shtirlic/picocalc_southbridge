# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#   2025-07-19 - Removed I2C_REGS_COMPAT option
#   2025-07-12 - Windows mkdir/rmdir tweaks
#   2025-05-09 - Added I2C_REGS_COMPAT option
#   2025-04-25 - Tuned for PicoCalc firmware project
#   2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = picocalc_BIOS_jcs


######################################
# building variables
######################################
# debug build?
DEBUG = 0
# optimization
OPT = -O3
# custom specs
#SPECS = picolibc.specs

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32f1xx_it.c \
Core/Src/hal_interface.c \
Core/Src/axp2101.c \
Core/Src/backlight.c \
Core/Src/batt.c \
Core/Src/eeprom.c \
Core/Src/fifo.c \
Core/Src/keyboard.c \
Core/Src/i2cs.c \
Core/Src/regs.c \
Core/Src/rtc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_crc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_wwdg.c \
Core/Src/system_stm32f1xx.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi
FLOAT-ABI = -mfloat-abi=soft

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32F103xB

ifeq ($(DEBUG), 1)
C_DEFS += -DDEBUG
endif

ifeq ($(I2C_REGS_COMPAT), 1)
C_DEFS += -DI2C_REGS_COMPAT
endif


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS_Device_ST_STM32F1xx/Include \
-IDrivers/CMSIS/Include

SPECS ?= nano.specs

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -std=gnu17 -Wall -Wextra -fdata-sections -ffunction-sections --specs=$(SPECS)

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -std=gnu17 -Wall -Wextra -pedantic -Wconversion -fdata-sections -ffunction-sections --specs=$(SPECS)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS_ASM := $(CFLAGS)
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103XX_FLASH.ld

# libraries
LIBS = 
LIBDIR = 
LDFLAGS = $(MCU) --specs=$(SPECS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -static

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# Verbose flag
ifeq ($(VERBOSE),1)
V		:=
else
V		:= @
endif

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS_ASM = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.s)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "  CC    $<"
	$(V)$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.s: %.c Makefile | $(BUILD_DIR) 
	@$(CC) $(CFLAGS_ASM) -w -fverbose-asm -S $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "  AS    $<"
	$(V)$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(V)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(OBJECTS_ASM) Makefile
	@echo "  LD    $<"
	$(V)$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(V)$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p "$@"

#######################################
# clean up
#######################################
clean:
	@echo "  CLEAN"
	rm -rf "$(BUILD_DIR)"
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
