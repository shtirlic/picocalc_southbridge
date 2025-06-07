#include "stm32f1xx_hal.h"


#ifndef REGS_H_
#define REGS_H_

enum reg_id {
	REG_ID_TYP = 0x00,			//!< firmware type (0=official, others=custom)
	REG_ID_VER = 0x01,			//!< fw version (7:4=Major, 3:0=Minor)
#ifdef I2C_REGS_COMPAT
	REG_ID_SYS_CFG = 0x02, // config
	REG_ID_INT = 0x03, // interrupt status
	REG_ID_KEY = 0x04, // key status
	REG_ID_BKL = 0x05, // backlight steps (0-9)
	REG_ID_DEB = 0x06, // debounce cfg
	REG_ID_FRQ = 0x07, // poll freq cfg
	REG_ID_PWR_CTRL = 0x08, // Power control (0: idle, 1: pico reset, 2: system reset, 3: reserved, 4: sleep, 5: full-shutdown)
	REG_ID_FIF = 0x09, // fifo
	REG_ID_BK2 = 0x0A, // keyboard backlight (0-9)
	REG_ID_BAT = 0x0B, // battery
	REG_ID_C64_MTX = 0x0C,// read c64 matrix
	REG_ID_C64_JS = 0x0D, // joystick io bits
	REG_ID_INT_CFG = 0x0E, // IRQ config
	REG_ID_RTC_CFG = 0x0F, // RTC general config
	REG_ID_RTC_DATE = 0x10, // RTC date
	REG_ID_RTC_TIME = 0x11, // RTC time
	REG_ID_RTC_ALARM_DATE = 0x12, // RTC alarm date
	REG_ID_RTC_ALARM_TIME = 0x13, // RTC alarm time
#else
	// TODO: REG_ID_CFG_0 - 32b (RW)
	REG_ID_SYS_CFG = 0x02,			//!< config
	REG_ID_INT_CFG = 0x03,			//!< IRQ config
	REG_ID_PWR_CTRL = 0x04,			//!< Power control (0: idle, 1: pico reset, 2: system reset, 3: reserved, 4: sleep, 5: full-shutdown)
	REG_ID_RTC_CFG = 0x05,			//!< RTC general config

	// TODO: REG_ID_CFG_1 - 32b (RW)
	REG_ID_DEB = 0x06,				//!< debounce cfg (time in ms)
	REG_ID_FRQ = 0x07,				//!< poll freq cfg (time in ms)
	REG_ID_BKL = 0x08,				//!< backlight steps (0-9)
	REG_ID_BK2 = 0x09,				//!< keyboard backlight (0-9)

	// TODO: REG_ID_RTC_DATE - 32b (RW)
	REG_ID_RTC_DATE = 0x0A,			//!< RTC date
	// TODO: REG_ID_RTC_TIME - 32b (RW)
	REG_ID_RTC_TIME = 0x0B,			//!< RTC time
	// TODO: REG_ID_RTC_ALARM_DATE - 32b (RW)
	REG_ID_RTC_ALARM_DATE = 0x0C,	//!< RTC alarm date
	// TODO: REG_ID_RTC_ALARM_TIME - 32b (RW)
	REG_ID_RTC_ALARM_TIME = 0x0D,	//!< RTC alarm time

	// TODO: REG_ID_INT - 32b (RO)
	REG_ID_INT = 0x10,				//!< interrupt flags status

	// TODO: REG_ID_KBD - 32b (RO)
	REG_ID_KEY = 0x14,				//!< key status - 8b
	REG_ID_C64_JS = 0x15,			//!< joystick io bits - 8b
	REG_ID_FIF = 0x16,				//!< fifo - 16b

	// TODO: REG_ID_BAT - 32b (RO)
	REG_ID_BAT = 0x18,				//!< battery percentage - 16b
	//REG_ID_BAT_RAW				//!< battery voltage value in mV - 16b

	// TODO: REG_ID_C64_MTX_0 - 32b (RO)
	REG_ID_C64_MTX = 0x1A,			//!< read c64 matrix
	// TODO: REG_ID_C64_MTX_1 - 32b (RO)
	// TODO: REG_ID_C64_MTX_2 - 32b (RO)
#endif
	REG_ID_LAST
};

#define REGS_GLOBAL_ENTRY()

typedef struct {
	const uint8_t addr;
	uint8_t* value[];
} REGS_GLOBAL_ENTRY;

#define CFG_OVERFLOW_ON		(1 << 0) //When a FIFO overflow happens, should the new entry still be pushed, overwriting the oldest one. If 0 then new entry is lost.
#define CFG_REPORT_MODS		(1 << 6) // Should Alt, Sym and Shifts be reported as well
#define CFG_USE_MODS		(1 << 7) // Should Alt, Sym and Shifts modify the keys reported
// CFG_STICKY_MODS // Pressing and releasing a mod affects next key pressed

#define INT_OVERFLOW		(1 << 0)
#define INT_CAPSLOCK		(1 << 1)
#define INT_NUMLOCK			(1 << 2)
#define INT_KEY				(1 << 3)
#define INT_PANIC			(1 << 4)

#define KEY_CAPSLOCK		(1 << 5)
#define KEY_NUMLOCK			(1 << 6)
#define KEY_COUNT_MASK		0x1F  //0x1F == 31

#define RTC_CFG_RUN_ALARM			(1 << 0)	// b0: Set the RTC alarm active.
#define RTC_CFG_REARM				(1 << 1)	// b1: If set, the RTC alarm will rearm for the next day trigger (if RTC_CFG_DATE_ALARM is set, repeat every day after the target date is reached)
#define RTC_CFG_DATE_ALARM			(1 << 2)	// b2: If set, check when alarm trig for the precise date, otherwise return to normal behavior (or sleep if wake-up)
#define RTC_CFG_PBTN_ALARM_IGNORE	(1 << 3)	// b3: If unset, override the power button shutdown behavior to keep the STM32 "alive" for the RTC to operate.
//#define RTC_CFG_SLEEP_MODE			(1 << 4)	// 0 = normal sleep (pico is shutdown, STM32 enter in low power state ~1.2mA); 1 = deep sleep (pico is shutdown, STM32 enter a stop state ~24uA)


uint8_t reg_get_value(enum reg_id reg);
uint8_t* reg_raw_access(void);
void reg_set_value(enum reg_id reg, uint8_t value);
uint8_t reg_is_bit_set(enum reg_id reg, uint8_t bit);
void reg_set_bit(enum reg_id reg, uint8_t bit);
void reg_init(void);
uint32_t reg_check_and_save_eeprom(void);
void reg_sync(void);

#endif /* REGS_H_ */
