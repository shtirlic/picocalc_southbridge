#include "batt.h"
#include "main.h"
#include "axp2101.h"


static uint32_t low_bat_count = 0;

void show_bat_segs(void) {
	if (AXP2101_isBatteryConnect() == 0)
		return;

	uint8_t pcnt;
	if (AXP2101_getBatteryPercent(&pcnt) != HAL_OK)
		return;
	uint8_t prev_state = (LL_GPIO_IsOutputPinSet(SYS_LED_GPIO_Port, SYS_LED_Pin) == 0);
	uint8_t blink_cnt;

	if(pcnt > 0 && pcnt < 33)
		blink_cnt = 1;
	else if(pcnt >= 33 && pcnt < 66)
		blink_cnt = 1;
	else if(pcnt >= 66 && pcnt <= 100)
		blink_cnt = 1;

	flash_one_time(blink_cnt, prev_state);

	if (AXP2101_isCharging())
		start_chg();
}

// CAUTION: This is related to the battery charging and discharging logic. If you're not sure what you're doing, please don't modify it, as it could damage the battery.
void low_bat(void) {
	uint8_t pcnt;
	if (AXP2101_getBatteryPercent(&pcnt) != HAL_OK)
		return;

	if ((pcnt >= 0) && (pcnt <= (uint8_t)LOW_BAT_VAL)) {
		low_bat_count++;
		LL_GPIO_SetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);

		if (pcnt <= 1) {
			AXP2101_setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
			if(pcnt == 0 && low_bat_count >= 4)
				AXP2101_shutdown();
		} else {
			AXP2101_setChargingLedMode(XPOWERS_CHG_LED_ON);
		}
	} else {
		low_bat_count = 0;
		LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
		AXP2101_setChargingLedMode(XPOWERS_CHG_LED_OFF);
	}
}

void start_chg(void) {
	LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
	AXP2101_setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
}

void stop_chg(void) {
	AXP2101_setChargingLedMode(XPOWERS_CHG_LED_OFF);
	low_bat();
}
