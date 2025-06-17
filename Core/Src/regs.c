#include "regs.h"

#include "hal_interface.h"
#include "stm32f1xx_hal_rtc_ex.h"
#include "eeprom.h"
#include "version.h"


extern RTC_HandleTypeDef hrtc;

static uint8_t regs[REG_ID_LAST] = {0};
static uint32_t eeprom_refresh_counter;

static uint32_t regs_unsync[(REG_ID_LAST / 32) + 1] = {0};
#define REGS_UNSYNC_SET(x)	regs_unsync[(x / 32)] |= (uint32_t)(1U << (x % 32))
#define REGS_UNSYNC_UNSET(x)	regs_unsync[(x / 32)] &= ~((uint32_t)(1U << (x % 32)))
#define REGS_UNSYNC_GET(x)	((regs_unsync[(x / 32)] >> (x % 32)) & (uint32_t)0x1)

inline uint8_t reg_get_value(enum reg_id reg) {
	if (reg >= REG_ID_LAST)
		return 0;

	return regs[reg];
}

inline uint8_t* reg_raw_access(void) {
	return regs;
}

inline void reg_set_value(enum reg_id reg, uint8_t value) {
	if (reg >= REG_ID_LAST)
		return;

	regs[reg] = value;
	REGS_UNSYNC_SET(reg);
	eeprom_refresh_counter = uptime_ms();
}

inline uint8_t reg_is_bit_set(enum reg_id reg, uint8_t bit) {
	if (reg >= REG_ID_LAST)
			return 0;

	return regs[reg] & bit;
}

inline void reg_set_bit(enum reg_id reg, uint8_t bit) {
	if (reg >= REG_ID_LAST)
			return;

	regs[reg] |= bit;
	REGS_UNSYNC_SET(reg);
	eeprom_refresh_counter = uptime_ms();
}

/*
 * | Bit    | Name             | Description                                                        |
 * | ------ |:----------------:| ------------------------------------------------------------------:|
 * | 7      | CFG_USE_MODS     | Should Alt, Sym and the Shift keys modify the keys being reported. |
 * | 6      | CFG_REPORT_MODS  | Should Alt, Sym and the Shift keys be reported as well.            |
 * | 5      | CFG_PANIC_INT    | Currently not implemented.                                         |
 * | 4      | CFG_KEY_INT      | Should an interrupt be generated when a key is pressed.            |
 * | 3      | CFG_NUMLOCK_INT  | Should an interrupt be generated when Num Lock is toggled.         |
 * | 2      | CFG_CAPSLOCK_INT | Should an interrupt be generated when Caps Lock is toggled.        |
 * | 1      | CFG_OVERFLOW_INT | Should an interrupt be generated when a FIFO overflow happens.     |
 * | 0      | CFG_OVERFLOW_ON  | When a FIFO overflow happens, should the new entry still be pushed,|
 * |        |                  | overwriting the oldest one. If 0 then new entry is lost.           |
 */
void reg_init(void) {
	uint32_t buff;

	regs[REG_ID_VER] = (uint8_t)((VERSION_MAJOR << 4) | VERSION_MINOR);	// 1.2 => (0x1 << 4) | 0x2

	EEPROM_ReadVariable(EEPROM_VAR_CFG, (EEPROM_Value*)&buff);
	regs[REG_ID_SYS_CFG] = (uint8_t)((buff >> 8) & 0xFF);
	regs[REG_ID_INT_CFG] = (uint8_t)(buff & 0xFF);

	EEPROM_ReadVariable(EEPROM_VAR_KBD, (EEPROM_Value*)&buff);
	//regs[REG_ID_DEB] = (uint8_t)((buff >> 8) & 0xFF);
	regs[REG_ID_FRQ] = (uint8_t)(buff & 0xFF);

	EEPROM_ReadVariable(EEPROM_VAR_BCKL, (EEPROM_Value*)&buff);
	regs[REG_ID_BKL] = (uint8_t)((buff >> 8) & 0xFF);
	regs[REG_ID_BK2] = (uint8_t)(buff & 0xFF);

	buff = 0;
	buff |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	buff |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3) << 16;
	regs[REG_ID_RTC_CFG] = (uint8_t)(buff & 0xFF);
	rtc_alarm_time.raw = (uint8_t)((buff >> 8) & 0xFFFFFF);

	buff = 0;
	buff |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
	buff |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5) << 16;
	rtc_alarm_date.raw = buff;

	regs[REG_ID_BAT] = 0; //default .no battery ,no charging

	regs[REG_ID_TYP] = 0xCA;	// That's me :3

	eeprom_refresh_counter = uptime_ms();
}

uint32_t reg_check_and_save_eeprom(void) {
	uint32_t result = EEPROM_SUCCESS;
	uint8_t need_save = 0;

	for (size_t i = 0; i < REG_ID_LAST; i++)
		if (REGS_UNSYNC_GET(i) == 1) {
			need_save = 1;
			break;
		}

	if (need_save == 1) {
		if (REGS_UNSYNC_GET(REG_ID_SYS_CFG) == 1)
			result |= EEPROM_WriteVariable(EEPROM_VAR_CFG, (EEPROM_Value)(uint16_t)((regs[REG_ID_SYS_CFG] << 8) | regs[REG_ID_INT_CFG]), EEPROM_SIZE16);

		if (REGS_UNSYNC_GET(REG_ID_DEB) == 1 || REGS_UNSYNC_GET(REG_ID_FRQ) == 1)
			result |= EEPROM_WriteVariable(EEPROM_VAR_KBD, (EEPROM_Value)(uint32_t)((keyboard_get_hold_period() << 16) | regs[REG_ID_FRQ]), EEPROM_SIZE32);

		if (REGS_UNSYNC_GET(REG_ID_BKL) == 1 || REGS_UNSYNC_GET(REG_ID_BK2) == 1)
			result |= EEPROM_WriteVariable(EEPROM_VAR_BCKL, (EEPROM_Value)(uint16_t)((regs[REG_ID_BKL] << 8) | regs[REG_ID_BK2]), EEPROM_SIZE16);

		if (REGS_UNSYNC_GET(REG_ID_RTC_ALARM_TIME) == 1 || REGS_UNSYNC_GET(REG_ID_RTC_CFG) == 1) {
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ((rtc_alarm_time.raw & 0xFF) << 8) | regs[REG_ID_RTC_CFG]);
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, rtc_alarm_time.raw >> 16);
		}

		if (REGS_UNSYNC_GET(REG_ID_RTC_ALARM_DATE) == 1) {
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, rtc_alarm_date.raw & 0xFFFF);
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, rtc_alarm_date.raw >> 16);
		}

		for (size_t i = 0; i < REG_ID_LAST; i++)
			REGS_UNSYNC_UNSET(i);
	}

	return result;
}

void reg_sync(void) {
	// Save user registers in EEPROM if unsynced every 1.5s
	if (uptime_ms() - eeprom_refresh_counter > 1500) {
		reg_check_and_save_eeprom();
		eeprom_refresh_counter = uptime_ms();
	}
}
