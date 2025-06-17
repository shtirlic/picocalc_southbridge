/**
  ******************************************************************************
  *
  * Copyright (c) 2025 C.ARE (JackCarterSmith).
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  *
  * SYS_LED and COL_x are open-drain, output logic is inverted.
  *
  * Unless requested by the user through REG_ID_PWR_CTRL register or by removing
  * the batteries, pressing the power button when running now put the STM32 in
  * low-power stop mode and shutdown the PICO board.
  * It's the only mean to keep the RTC functional.
  * A full shutdown using AXP2101 power-off or by removing the batteries will
  * reset the RTC and disable auto wake-up feature.
  *
  */

#include "hal_interface.h"
#include "axp2101.h"
#include "backlight.h"
#include "batt.h"
#include "eeprom.h"
#include "fifo.h"
#include "keyboard.h"
#include "i2cs.h"
#include "regs.h"
#include "rtc.h"


// Private define ------------------------------------------------------------
//#define DEFAULT_LCD_BL	(205)	// ~40% PWM@7.81kHz (9 bits resolution)
//#define DEFAULT_KBD_BL	(20)	// ~4% PWM@7.81kHz (9 bits resolution)
#define DEFAULT_LCD_BL		(3)		//step-4 (~50%)
#define DEFAULT_KBD_BL		(0)		//step-1 (0%)
#define DEFAULT_KBD_FREQ	(KEY_POLL_TIME)
#define DEFAULT_KBD_DEB		(KEY_HOLD_TIME)
#define DEFAULT_RCT_CFG		(0x00)

#ifdef DEBUG
#define DEBUG_UART_MSG(msg)			HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 1000)
//#define DEBUG_UART_MSG2(d,s)		HAL_UART_Transmit(&huart1, (uint8_t*)d, s, 200)
#define DEBUG_UART_MSG2(d,sz, swp)	uart_rawdata_write(d,sz,swp)
#endif


// Private typedef -----------------------------------------------------------


// Private variables ---------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern RTC_HandleTypeDef hrtc;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#ifdef DEBUG
extern UART_HandleTypeDef huart1;
static const uint8_t hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
#endif
#ifdef UART_PICO_INTERFACE
extern UART_HandleTypeDef huart3;
#endif

volatile uint32_t systicks_counter = 0;		// 1 MHz systick counter
static uint32_t pmu_check_counter = 0;

static uint8_t keycb_start = 0;
static uint32_t head_phone_status = 0;	// TODO: Combine status registers

volatile uint8_t stop_mode_active = 0;

volatile uint8_t pmu_irq = 0;
static uint32_t pmu_online = 0;


// Private variables ---------------------------------------------------------
//static void lock_cb(uint8_t caps_changed, uint8_t num_changed);
static void key_cb(char key, enum key_state state);
static void hw_check_HP_presence(void);
static void sync_bat(void);
#ifdef DEBUG
static void printPMU(void);
#endif
static void check_pmu_int(void);
static void rtc_ctrl_reg_check(void);
static void pwr_ctrl_reg_check(void);
static void sys_prepare_sleep(void);
static void sys_wake_sleep(void);
static void sys_stop_pico(void);
static void sys_start_pico(void);

extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		systicks_counter += 1;
	}
}

#ifdef DEBUG
void uart_rawdata_write(uint32_t c, size_t s, uint8_t swap) {
	uint8_t r[4];
	uint32_t v = swap ? __REV(c) : c;

	HAL_UART_Transmit(&huart1, (uint8_t*)"0x", 2, 40);
	for (size_t i = 0; i < s; i++) {
		uint8_t index = swap ? (uint8_t)(4-s+i) : (uint8_t)i;
		r[0] = hexmap[(((uint8_t*)&v)[index] & 0xF0) >> 4];
		r[1] = hexmap[((uint8_t*)&v)[index] & 0x0F];
		HAL_UART_Transmit(&huart1, r, 2, 40);
	}
}
#endif


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	uint32_t result = 0;

	// Initialize the STM32 HAL system
	result = HAL_Interface_init();
	if (result != HAL_OK)
		Error_Handler();

	// Start the systick timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
		Error_Handler();

	LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);	// I'm alive!

	// EEPROM emulation init
	if (EEPROM_Init() != EEPROM_SUCCESS)
		Error_Handler();
#ifdef DEBUG
	DEBUG_UART_MSG("EEPROM init\n\r");
#endif

	// Check EEPROM first run
	EEPROM_ReadVariable(EEPROM_VAR_ID, (EEPROM_Value*)&result);
	if ((uint16_t)result != 0xCA1C) {
		EEPROM_WriteVariable(EEPROM_VAR_BCKL, (EEPROM_Value)(uint16_t)((DEFAULT_LCD_BL << 8) | DEFAULT_KBD_BL), EEPROM_SIZE16);
		EEPROM_WriteVariable(EEPROM_VAR_KBD, (EEPROM_Value)(uint32_t)((DEFAULT_KBD_DEB << 16) | DEFAULT_KBD_FREQ), EEPROM_SIZE32);
		EEPROM_WriteVariable(EEPROM_VAR_CFG, (EEPROM_Value)(uint16_t)(((CFG_USE_MODS | CFG_REPORT_MODS) << 8) | (INT_OVERFLOW | INT_KEY)), EEPROM_SIZE16);
		EEPROM_WriteVariable(EEPROM_VAR_ID, (EEPROM_Value)(uint16_t)0xCA1C, EEPROM_SIZE16);
#ifdef DEBUG
		DEBUG_UART_MSG("EEPROM first start!\n\r");
#endif
	}

	// Check RTC SRAM first run
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0) {
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xCA1C);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ((rtc_alarm_time.raw & 0xFF) << 8) | DEFAULT_RCT_CFG);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, rtc_alarm_time.raw >> 16);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, rtc_alarm_date.raw & 0xFFFF);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, rtc_alarm_date.raw >> 16);
	}

	// I2C-Pico interface registers
	reg_init();
	HAL_Delay(10);
	if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
		Error_Handler();

	// Check for AXP2101 is accessible on secondary I2C bus
	result = 0;
	HAL_I2C_Mem_Read(&hi2c2, 0x68, XPOWERS_AXP2101_IC_TYPE, 1, (uint8_t*)&result, 1, 60);
	if (result == XPOWERS_AXP2101_CHIP_ID) {
#ifdef DEBUG
		DEBUG_UART_MSG("PMU ID: ");
		DEBUG_UART_MSG2((uint32_t)result, 1, 0);
		DEBUG_UART_MSG("\n\r");
#endif
		pmu_online = 1;
	}
#ifdef DEBUG
	else {
		DEBUG_UART_MSG("PMU not online!\n\r");
	}
#endif

	// Start LCD and KBD backlight PWM
	lcd_backlight_off();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	kbd_backlight_on();
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();
#ifdef DEBUG
	DEBUG_UART_MSG("Bckl params: ");
	DEBUG_UART_MSG2(((uint32_t)result >> 8), 1, 1);
	DEBUG_UART_MSG(", ");
	DEBUG_UART_MSG2(((uint32_t)result & 0xFF), 1, 1);
	DEBUG_UART_MSG("\n\r");
#endif
	keyboard_set_key_callback(key_cb);

	// PICO MCU start
	sys_start_pico();
#ifdef DEBUG
	DEBUG_UART_MSG("Pico started\n\r");
#endif

	HAL_Delay(500);
	lcd_backlight_on();

	// It is necessary to disable the detection function of the TS pin on the
	// board without the battery temperature detection function, otherwise it will
	// cause abnormal charging
	AXP2101_setSysPowerDownVoltage(2800);
	AXP2101_disableTSPinMeasure();
	// AXP2101_enableTemperatureMeasure();
	AXP2101_enableBattDetection();
	AXP2101_enableVbusVoltageMeasure();
	AXP2101_enableBattVoltageMeasure();
	AXP2101_enableSystemVoltageMeasure();

	AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

	AXP2101_disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
	AXP2101_clearIrqStatus();
	AXP2101_enableIRQ(XPOWERS_AXP2101_BAT_INSERT_IRQ |
					XPOWERS_AXP2101_BAT_REMOVE_IRQ |  // BATTERY
					XPOWERS_AXP2101_WARNING_LEVEL1_IRQ |
					XPOWERS_AXP2101_VBUS_INSERT_IRQ |
					XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
					XPOWERS_AXP2101_PKEY_SHORT_IRQ |
					XPOWERS_AXP2101_PKEY_LONG_IRQ |  // POWER KEY
					XPOWERS_AXP2101_BAT_CHG_DONE_IRQ |
					XPOWERS_AXP2101_BAT_CHG_START_IRQ  // CHARGE
	);
	// setLowBatWarnThreshold Range:  5% ~ 20%
	// The following data is obtained from actual testing , Please see the description below for the test method.
	// 20% ~= 3.7v
	// 15% ~= 3.6v
	// 10% ~= 3.55V
	// 5%  ~= 3.5V
	// 1%  ~= 3.4V
	AXP2101_setLowBatWarnThreshold(20); // Set to trigger interrupt when reaching 20%

	// setLowBatShutdownThreshold Range:  0% ~ 15%
	// The following data is obtained from actual testing , Please see the description below for the test method.
	// 15% ~= 3.6v
	// 10% ~= 3.55V
	// 5%  ~= 3.5V
	// 1%  ~= 3.4V
	AXP2101_setLowBatShutdownThreshold(5);  //This is related to the battery charging and discharging logic. If you're not sure what you're doing, please don't modify it, as it could damage the battery.

	keycb_start = 1;
	sync_bat();
	low_bat();

	while (1) {
//#ifndef DEBUG
//		LL_IWDG_ReloadCounter(IWDG);
//#endif

		// Re-arm I2CS in case of lost master signal
		if (i2cs_state != I2CS_STATE_IDLE && ((uptime_ms() - i2cs_rearm_counter) > I2CS_REARM_TIMEOUT))
			i2cs_state = I2CS_STATE_IDLE;

		reg_sync();
		check_pmu_int();
		keyboard_process();
		hw_check_HP_presence();
		rtc_ctrl_reg_check();
		pwr_ctrl_reg_check();

		// Execute stop/sleep mode if requested
		if (stop_mode_active == 1) {
			/* Prepare peripherals to the low-power mode */
			sys_stop_pico();
			sys_prepare_sleep();

			/* Low-power mode entry */
			//HAL_WWDG_Disable();
			HAL_SuspendTick();
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			SystemClock_Config();
			HAL_ResumeTick();
			HAL_Delay(300);

			/* Wake-up peripherals from low-power mode */
			sys_wake_sleep();
			sys_start_pico();
		}
	}
}


/*
static void lock_cb(uint8_t caps_changed, uint8_t num_changed) {
	uint8_t int_trig = 0;

	if (caps_changed && reg_is_bit_set(REG_ID_CFG, CFG_CAPSLOCK_INT)) {
		reg_set_bit(REG_ID_INT, INT_CAPSLOCK);
		int_trig = 1;
	}

	if (num_changed && reg_is_bit_set(REG_ID_CFG, CFG_NUMLOCK_INT)) {
		reg_set_bit(REG_ID_INT, INT_NUMLOCK);
		int_trig = 1;
	}

#ifndef UART_PICO_INTERFACE
	if (int_trig == 1)
		LL_GPIO_ResetOutputPin(PICO_IRQ_GPIO_Port, PICO_IRQ_Pin);		// Assert the IRQ signal to the pico
#endif
}
*/

static void key_cb(char key, enum key_state state) {
	uint8_t int_trig = 0;

	if (keycb_start == 0) {
		fifo_flush();
		return;
	}

	if (reg_is_bit_set(REG_ID_INT_CFG, INT_KEY)) {
		reg_set_bit(REG_ID_INT, INT_KEY);
		int_trig = 1;
	}

#ifdef DEBUG
	DEBUG_UART_MSG("key: ");
	DEBUG_UART_MSG2(key, 1, 0);
	DEBUG_UART_MSG("\n\r");
	DEBUG_UART_MSG("state: ");
	DEBUG_UART_MSG2(state, 1, 0);
	//DEBUG_UART_MSG(" blk: ");
	//DEBUG_UART_MSG2(reg_get_value(REG_ID_BKL), 1, 0);
	DEBUG_UART_MSG("\n\r");
#endif

	const struct fifo_item item = {key, state};
	if (!fifo_enqueue(item)) {
		if (reg_is_bit_set(REG_ID_INT_CFG, INT_OVERFLOW)) {
			reg_set_bit(REG_ID_INT, INT_OVERFLOW);  // INT_OVERFLOW  The interrupt was generated by FIFO overflow.
			int_trig = 1;
		}

		if (reg_is_bit_set(REG_ID_SYS_CFG, CFG_OVERFLOW_ON)) fifo_enqueue_force(item);
	}

#ifndef UART_PICO_INTERFACE
	if (int_trig == 1)
		LL_GPIO_ResetOutputPin(PICO_IRQ_GPIO_Port, PICO_IRQ_Pin);		// Assert the IRQ signal to the pico
#endif
}

__STATIC_INLINE void hw_check_HP_presence(void) {
	uint32_t v = LL_GPIO_IsInputPinSet(HP_DET_GPIO_Port, HP_DET_Pin);

	if (v != head_phone_status) {
		if (v != 0) {
#ifdef DEBUG
			DEBUG_UART_MSG("HeadPhone inserted\n\r");
#endif
			LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
		} else {
#ifdef DEBUG
			DEBUG_UART_MSG("HeadPhone removed\n\r");
#endif
			LL_GPIO_SetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
		}

		head_phone_status = v;
	}
}

__STATIC_INLINE void sync_bat(void) {
	uint8_t pcnt;
	if (AXP2101_getBatteryPercent(&pcnt) != HAL_OK)
		return;
#ifdef DEBUG
	DEBUG_UART_MSG("check_pmu_int: ");
	DEBUG_UART_MSG2((uint32_t)pcnt, 1, 0);
	DEBUG_UART_MSG("\n\r");
#endif

	if (pcnt > 100) {  // disconnect
		pcnt = 0;
	} else {  // battery connected
		if (AXP2101_isCharging())
			pcnt |= (1 << 7);
		low_bat();
	}

	reg_set_value(REG_ID_BAT, pcnt);
}

#ifdef DEBUG
__STATIC_INLINE void printPMU(void) {
	DEBUG_UART_MSG("PMU isCharging: ");
	if (AXP2101_isCharging())
		DEBUG_UART_MSG("YES\n\r");
	else
		DEBUG_UART_MSG( "NO\n\r");

	DEBUG_UART_MSG("PMU isDischarge: ");
	if (AXP2101_isDischarge())
		DEBUG_UART_MSG("YES\n\r");
	else
		DEBUG_UART_MSG( "NO\n\r");

	DEBUG_UART_MSG("PMU isStandby: ");
	if (AXP2101_isStandby())
		DEBUG_UART_MSG("YES\n\r");
	else
		DEBUG_UART_MSG( "NO\n\r");

	DEBUG_UART_MSG("PMU isVbusIn: ");
	if (AXP2101_isVbusIn())
		DEBUG_UART_MSG("YES\n\r");
	else
		DEBUG_UART_MSG( "NO\n\r");

	DEBUG_UART_MSG("PMU isVbusGood: ");
	if (AXP2101_isVbusGood())
		DEBUG_UART_MSG("YES\n\r");
	else
		DEBUG_UART_MSG( "NO\n\r");

	DEBUG_UART_MSG("PMU getChargerStatus: ");
	uint8_t charge_status = AXP2101_getChargerStatus();
	if (charge_status == XPOWERS_AXP2101_CHG_TRI_STATE) {
		DEBUG_UART_MSG("tri_charge");
	} else if (charge_status == XPOWERS_AXP2101_CHG_PRE_STATE) {
		DEBUG_UART_MSG("pre_charge");
	} else if (charge_status == XPOWERS_AXP2101_CHG_CC_STATE) {
		DEBUG_UART_MSG("constant charge");
	} else if (charge_status == XPOWERS_AXP2101_CHG_CV_STATE) {
		DEBUG_UART_MSG("constant voltage");
	} else if (charge_status == XPOWERS_AXP2101_CHG_DONE_STATE) {
		DEBUG_UART_MSG("charge done");
	} else if (charge_status == XPOWERS_AXP2101_CHG_STOP_STATE) {
		DEBUG_UART_MSG("not charging");
	}

	DEBUG_UART_MSG("PMU getBattVoltage: ");
	DEBUG_UART_MSG2(AXP2101_getBattVoltage(), 2, 0);
	DEBUG_UART_MSG("mV\n\r");
	DEBUG_UART_MSG("PMU getVbusVoltage: ");
	DEBUG_UART_MSG2(AXP2101_getVbusVoltage(), 2, 0);
	DEBUG_UART_MSG("mV\n\r");
	DEBUG_UART_MSG("PMU getSystemVoltage: ");
	DEBUG_UART_MSG2(AXP2101_getSystemVoltage(), 2, 0);
	DEBUG_UART_MSG("mV\n\r");

	// The battery percentage may be inaccurate at first use, the PMU will
	// automatically learn the battery curve and will automatically calibrate the
	// battery percentage after a charge and discharge cycle
	if (AXP2101_isBatteryConnect()) {
		DEBUG_UART_MSG("PMU getBatteryPercent: ");
		uint8_t pcnt = 0;
		AXP2101_getBatteryPercent(&pcnt);
		DEBUG_UART_MSG2(pcnt, 1, 0);
		DEBUG_UART_MSG("%\n\r");
	}
}
#endif

__STATIC_INLINE void check_pmu_int(void) {
	if (!pmu_online)
		return;

	uint8_t pcnt;

	if (uptime_ms() - pmu_check_counter > 20000) {
		pmu_check_counter = uptime_ms();  // reset time
		AXP2101_getBatteryPercent(&pcnt);
#ifdef DEBUG
		DEBUG_UART_MSG("check_pmu_int: ");
		DEBUG_UART_MSG2((uint32_t)pcnt, 1, 0);
		DEBUG_UART_MSG("\n\r");
#endif

		if (pcnt > 100) {  // disconnect
			pcnt = 0;
		} else {  // battery connected
			if (AXP2101_isCharging())
				pcnt |= (1 << 7);
			low_bat();
		}

		reg_set_value(REG_ID_BAT,pcnt);
	}

	if (pmu_irq) {
		pmu_irq = 0;	// Reset interrupt flag

		// Get PMU Interrupt Status Register
		uint32_t status;
		AXP2101_getIrqStatus(&status);
#ifdef DEBUG
		DEBUG_UART_MSG("PMU IRQ status: ");
		DEBUG_UART_MSG2(status, 4, 1);
		DEBUG_UART_MSG("\n\r");
#endif

		/*
		// When the set low-voltage battery percentage warning threshold is reached,
		// set the threshold through getLowBatWarnThreshold( 5% ~ 20% )
		if (PMU.isDropWarningLevel2Irq()) {
		  Serial1.println("isDropWarningLevel2");
		  //report_bat();
		}
		*/

		// When the set low-voltage battery percentage shutdown threshold is reached
		// set the threshold through setLowBatShutdownThreshold()
		if (AXP2101_isDropWarningLevel1Irq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isDropWarningLevel1\n\r");
#endif
			//report_bat();
			//
			AXP2101_shutdown();
		}
		/*if (PMU.isVbusInsertIrq()) {
		  Serial1.println("isVbusInsert");
		}*/
		if (AXP2101_isVbusRemoveIrq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isVbusRemove\n\r");
#endif
			stop_chg();
		}

		if (AXP2101_isBatInsertIrq()) {
			AXP2101_getBatteryPercent(&pcnt);
			if (pcnt > 100) {  // disconnect
				pcnt = 0;
			} else {  // battery connected
				pcnt |= (1 << 7);
			}
			reg_set_value(REG_ID_BAT, pcnt);
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isBatInsert\n\r");
#endif
		}

		if (AXP2101_isBatRemoveIrq()) {
			reg_set_value(REG_ID_BAT,0);
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isBatRemove\n\r");
#endif
			stop_chg();
		}

		if (AXP2101_isPkeyShortPressIrq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isPekeyShortPress\n\r");

			uint8_t data[4] = {0};
			AXP2101_readDataBuffer(data, XPOWERS_AXP2101_DATA_BUFFER_SIZE);
			DEBUG_UART_MSG("PMU data buffer:\n\r");
			DEBUG_UART_MSG2(data[0], 1, 0);
			DEBUG_UART_MSG("\n\r");
			DEBUG_UART_MSG2(data[1], 1, 0);
			DEBUG_UART_MSG("\n\r");
			DEBUG_UART_MSG2(data[2], 1, 0);
			DEBUG_UART_MSG("\n\r");
			DEBUG_UART_MSG2(data[3], 1, 0);
			DEBUG_UART_MSG("\n\r");

			printPMU();
#endif
			if (stop_mode_active == 1) {
				stop_mode_active = 0;
			} else {
				if (keyboard_get_shift() && (reg_get_value(REG_ID_PWR_CTRL) == 0))
					reg_set_value(REG_ID_PWR_CTRL, PWR_CTRL_PICO_RST);
			}
		}

		if (AXP2101_isPkeyLongPressIrq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isPekeyLongPress\n\r");
#endif
			//Serial1.println("write pmu data buffer .");
			//uint8_t data[4] = {1, 2, 3, 4};
			//PMU.writeDataBuffer(data, XPOWERS_AXP2101_DATA_BUFFER_SIZE);

			if (stop_mode_active == 1) {
				stop_mode_active = 0;
			} else {
				AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
				stop_mode_active = 1;
			}
		}

		if (AXP2101_isBatChargeDoneIrq()) {
			AXP2101_getBatteryPercent(&pcnt);
			if (pcnt > 100) {  // disconnect
				pcnt = 0;
			} else {  // battery connected
				pcnt |= (1 << 7);
			}
			reg_set_value(REG_ID_BAT,pcnt);
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isBatChagerDone\n\r");
#endif
			stop_chg();
		}

		if (AXP2101_isBatChargeStartIrq()) {
			AXP2101_getBatteryPercent(&pcnt);
			if (pcnt > 100) {  // disconnect
				pcnt = 0;
			} else {  // battery connected
				pcnt |= (1 << 7);
			}
			reg_set_value(REG_ID_BAT,pcnt);
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isBatChagerStart\n\r");
#endif
			if(AXP2101_isBatteryConnect())
				start_chg();
		}

		// Clear PMU Interrupt Status Register
		AXP2101_clearIrqStatus();
	}
}

__STATIC_INLINE void rtc_ctrl_reg_check(void) {
	if (rtc_reg_xor_events != 0) {
		if ((rtc_reg_xor_events & RTC_CFG_RUN_ALARM) == RTC_CFG_RUN_ALARM) {
			if (reg_get_value(REG_ID_RTC_CFG) & RTC_CFG_RUN_ALARM) {
				if (rtc_run_alarm() != HAL_OK)
					reg_set_value(REG_ID_RTC_CFG, reg_get_value(REG_ID_RTC_CFG) & (uint8_t)~RTC_CFG_RUN_ALARM);
			} else {
				if (rtc_stop_alarm() != HAL_OK)
					reg_set_value(REG_ID_RTC_CFG, reg_get_value(REG_ID_RTC_CFG) | RTC_CFG_RUN_ALARM);
			}

			rtc_reg_xor_events &= (uint8_t)~RTC_CFG_RUN_ALARM;
		}
	}
}

__STATIC_INLINE void pwr_ctrl_reg_check(void) {
	switch (reg_get_value(REG_ID_PWR_CTRL)) {
	case PWR_CTRL_PICO_RST:
		reg_set_value(REG_ID_PWR_CTRL, 0);
		HAL_Delay(200);		// Wait for final I2C answer
		if (HAL_I2C_DisableListen_IT(&hi2c1) != HAL_OK)
			Error_Handler();

		sys_stop_pico();
		HAL_Delay(200);		// No need to use keyboard, so a simple delay should suffice
		if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
			if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
				Error_Handler();	//TODO: replace by a I2C reset request

		sys_start_pico();
		break;

	case PWR_CTRL_FULL_RST:
		reg_set_value(REG_ID_PWR_CTRL, 0);
		HAL_Delay(200);		// Wait for final I2C answer
		if (HAL_I2C_DisableListen_IT(&hi2c1) != HAL_OK)
			Error_Handler();
		sys_stop_pico();

		NVIC_SystemReset();
		break;

	//case PWR_CTRL_RESERVED:
	//	break;

	case PWR_CTRL_SLEEP:
		reg_set_value(REG_ID_PWR_CTRL, 0);
		sys_stop_pico();
		AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

		stop_mode_active = 1;
		break;

	case PWR_CTRL_SHUTDOWN:
		reg_set_value(REG_ID_PWR_CTRL, 0);
		sys_stop_pico();
		AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

		AXP2101_shutdown();		// Full shudown will rip the RTC configuration! Need to be reset at next reboot.
		break;

	default:
		break;
	}
}

__STATIC_INLINE void sys_prepare_sleep(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Put PMIC in low-trigger mode
	AXP2101_disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
	AXP2101_clearIrqStatus();
	AXP2101_enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ |
					XPOWERS_AXP2101_PKEY_LONG_IRQ |
					XPOWERS_AXP2101_WARNING_LEVEL1_IRQ
	);

	// Backlights shut-off
	lcd_backlight_off();
	kbd_backlight_off();
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	// Disable I2C slave
	HAL_I2C_DisableListen_IT(&hi2c1);

	// Front LED switched in low-power mode
	GPIO_InitStruct.Pin = SYS_LED_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);
	LL_GPIO_SetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
}

__STATIC_INLINE void sys_wake_sleep(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Restore front LED state
	GPIO_InitStruct.Pin = SYS_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);

	// Enable I2C slave
	if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
		if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
			Error_Handler();	//TODO: replace by a I2C reset request

	// Restart backlights PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_Delay(300);
	kbd_backlight_on();
	lcd_backlight_on();

	// Re-enable complete IRQ from PMIC
	AXP2101_enableIRQ(XPOWERS_AXP2101_BAT_INSERT_IRQ |
					XPOWERS_AXP2101_BAT_REMOVE_IRQ |  // BATTERY
					XPOWERS_AXP2101_VBUS_INSERT_IRQ |
					XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
					XPOWERS_AXP2101_BAT_CHG_DONE_IRQ |
					XPOWERS_AXP2101_BAT_CHG_START_IRQ  // CHARGE
	);
}

__STATIC_INLINE void sys_stop_pico(void) {
	LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);	// Disable speaker Amp. power
	LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);		// Disable PICO power
}

__STATIC_INLINE void sys_start_pico(void) {
	LL_GPIO_SetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);		// Enable PICO power
	LL_GPIO_SetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);	// Enable speaker Amp. power
}
