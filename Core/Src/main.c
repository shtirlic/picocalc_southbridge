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
  */

#include "hal_interface.h"
#include "cmsis_gcc.h"
#include "axp2101.h"
#include "backlight.h"
#include "batt.h"
#include "eeprom.h"
#include "fifo.h"
#include "keyboard.h"
#include "regs.h"


// Private define ------------------------------------------------------------
//#define DEFAULT_LCD_BL	(205)	// ~40% PWM@7.81kHz (9 bits resolution)
//#define DEFAULT_KBD_BL	(20)	// ~4% PWM@7.81kHz (9 bits resolution)
#define DEFAULT_LCD_BL		(3)		//step-4 (~50%)
#define DEFAULT_KBD_BL		(0)		//step-1 (0%)
#define DEFAULT_KBD_FREQ	(KEY_POLL_TIME)
#define DEFAULT_KBD_DEB		(KEY_HOLD_TIME)

#define I2CS_REARM_TIMEOUT	500
#define I2CS_W_BUFF_LEN		31+1	// The last one must be only a 0 value, TODO: another cleaner way?

#ifdef DEBUG
#define DEBUG_UART_MSG(msg)			HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 1000)
//#define DEBUG_UART_MSG2(d,s)		HAL_UART_Transmit(&huart1, (uint8_t*)d, s, 200)
#define DEBUG_UART_MSG2(d,sz, swp)	uart_rawdata_write(d,sz,swp)
#endif

#define STM32F1xxx_BL_ADDR	0x1FFFF000	// STM32 bootloader memory address


// Private typedef -----------------------------------------------------------
enum i2cs_state {
	//I2CS_STATE_HALT,
	I2CS_STATE_IDLE,
	I2CS_STATE_REG_REQUEST,
	I2CS_STATE_REG_ANSWER
};


// Private variables ---------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

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
static uint32_t i2cs_rearm_counter = 0;

static uint8_t i2cs_r_buff[5];
static volatile uint8_t i2cs_r_idx = 0;
static uint8_t i2cs_w_buff[I2CS_W_BUFF_LEN];
static volatile uint8_t i2cs_w_idx = 0;
static volatile uint8_t i2cs_w_len = 0;
static enum i2cs_state i2cs_state = I2CS_STATE_IDLE;

static uint8_t keycb_start = 0;
static uint32_t head_phone_status = 0;	// TODO: Combine status registers

volatile uint8_t pmu_irq = 0;
static uint32_t pmu_online = 0;

struct blvt_t {
    uint32_t __sp;
    void (*__bl_call)(void);
};
#define STM32F1xxx_BL_VECTOR_TABLE	((struct blvt_t *)STM32F1xxx_BL_ADDR)


// Private variables ---------------------------------------------------------
//static void lock_cb(uint8_t caps_changed, uint8_t num_changed);
static void reset_to_bootloader(void);
static void key_cb(char key, enum key_state state);
static void hw_check_HP_presence(void);
static void sync_bat(void);
#ifdef DEBUG
static void printPMU(void);
#endif
static void check_pmu_int(void);

extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		systicks_counter += 1;
	}
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (hi2c == &hi2c1) {
		// I2C slave addr match error detection
		if (AddrMatchCode != 0x3E)	// 0x1F << 1
			return;

		if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
			if (i2cs_state == I2CS_STATE_IDLE) {
				i2cs_state = I2CS_STATE_REG_REQUEST;

				i2cs_r_idx = 0;
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2cs_r_buff, 1, I2C_FIRST_FRAME);	// This write the first received byte to i2cs_r_buff[0]

				i2cs_rearm_counter = uptime_ms();
			}
		}

		if (TransferDirection == I2C_DIRECTION_RECEIVE) {
			if (i2cs_state == I2CS_STATE_REG_REQUEST) {
				const uint8_t is_write = (uint8_t)(i2cs_r_buff[0] & (1 << 7));
				const uint8_t reg = (uint8_t)(i2cs_r_buff[0] & ~(1 << 7));

				i2cs_w_buff[0] = reg;
				i2cs_w_len = 2;

				if (reg == REG_ID_BKL) {	// We wait an another byte for these registers
					if (is_write)
						lcd_backlight_update(i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_BKL);
				} else if (reg == REG_ID_BK2) {
					if (is_write)
						kbd_backlight_update(i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_BK2);
				} else if (reg == REG_ID_CFG) {
					if (is_write)
						reg_set_value(REG_ID_CFG, i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_CFG);
				} else if (reg == REG_ID_INT_CFG) {
					if (is_write)
						reg_set_value(REG_ID_INT_CFG, i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_INT_CFG);
				} else if (reg == REG_ID_DEB) {
					if (is_write) {
						keyboard_set_hold_period(*((uint16_t*)&i2cs_r_buff[1]));
						reg_set_value(REG_ID_DEB, 0);	// Trig async flag for EEPROM saving
					}
					*((uint16_t*)&i2cs_w_buff[1]) = keyboard_get_hold_period();
					i2cs_w_len = 3;
				} else if (reg == REG_ID_FRQ) {
					if (is_write)
						reg_set_value(REG_ID_FRQ, i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_FRQ);
				} else if (reg == REG_ID_FIF) {
					struct fifo_item item = {0};
					fifo_dequeue(&item);
					i2cs_w_buff[0] = item.state;
					i2cs_w_buff[1] = item.key;
				} else if (reg == REG_ID_INT) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_INT);
					LL_GPIO_SetOutputPin(PICO_IRQ_GPIO_Port, PICO_IRQ_Pin);	// De-assert the IRQ signal
				} else if (reg == REG_ID_VER) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_VER);
				} else if (reg == REG_ID_TYP) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_TYP);
				} else if (reg == REG_ID_BAT) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_BAT);
				} else if (reg == REG_ID_KEY) {
					i2cs_w_buff[0] = fifo_count();
					i2cs_w_buff[0] |= keyboard_get_numlock() ? KEY_NUMLOCK : 0x00;
					i2cs_w_buff[0] |= keyboard_get_capslock() ? KEY_CAPSLOCK : 0x00;
				} else if (reg == REG_ID_C64_MTX) {
					//memcpy(write_buffer + 1, io_matrix, sizeof(io_matrix));
					*((uint32_t*)(&i2cs_w_buff[1]) + 0) = *((uint32_t*)(io_matrix) + 0);
					*((uint32_t*)(&i2cs_w_buff[1]) + 1) = *((uint32_t*)(io_matrix) + 1);
					i2cs_w_buff[9] = io_matrix[8];
					i2cs_w_len = 10;
				} else if (reg == REG_ID_C64_JS) {
					i2cs_w_buff[1] = js_bits;
				} else if (reg == REG_ID_RST) {
					if (is_write)
						reg_set_value(REG_ID_RST, 1);
					i2cs_w_buff[1] = reg_get_value(REG_ID_RST);
				} else if (reg == REG_ID_RST_PICO) {
					if (is_write)
						reg_set_value(REG_ID_RST_PICO, 1);
					i2cs_w_buff[1] = reg_get_value(REG_ID_RST_PICO);
				} else if (reg == REG_ID_SHTDW) {
					if (is_write) {
						reg_set_value(REG_ID_SHTDW, 1);
						return;		// Ignore answer, everything will be shutdown
					}
					i2cs_w_buff[1] = 0;
				} else {
					i2cs_w_buff[0] = 0;
					i2cs_w_buff[1] = 0;
				}

				i2cs_state = I2CS_STATE_REG_ANSWER;
				i2cs_w_idx = 0;

				HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, i2cs_w_buff, i2cs_w_len, I2C_FIRST_AND_LAST_FRAME);

				i2cs_rearm_counter = uptime_ms();
			}
		}
	}
}

extern void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		i2cs_r_idx++;

		if (i2cs_state == I2CS_STATE_REG_REQUEST) {
			const uint8_t is_write = (uint8_t)(i2cs_r_buff[0] & (1 << 7));
			const uint8_t reg = (uint8_t)(i2cs_r_buff[0] & ~(1 << 7));
			uint8_t bytes_needed = 0;

			// Check for another mandatories bytes depending on register requested
			if (reg == REG_ID_BKL ||
				reg == REG_ID_BK2 ||
				reg == REG_ID_CFG ||
				reg == REG_ID_INT_CFG ||
				reg == REG_ID_FRQ) {
				if (is_write)
					bytes_needed = 1;

			} else if (reg == REG_ID_DEB) {
				if (is_write)
					bytes_needed = 2;
			}

			if (bytes_needed > 0)
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2cs_r_buff + i2cs_r_idx, bytes_needed, I2C_NEXT_FRAME);	// This write the second or more received byte to i2cs_r_buff[1]
		}
	}
}

extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		if (i2cs_state == I2CS_STATE_REG_ANSWER)
			i2cs_state = I2CS_STATE_IDLE;

		HAL_I2C_EnableListen_IT(hi2c);
	}
}

extern void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1)
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
			Error_Handler();
			// Actually this will trigger the watchdog and restart the system... That can ruin the day of the user.
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

	LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);	// I'm alive!

	// Start the systick timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
		Error_Handler();

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

	// Enable PICO power
	LL_GPIO_SetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);
#ifdef DEBUG
	DEBUG_UART_MSG("Pico started\n\r");
#endif

	// Enable speaker Amp. power
	LL_GPIO_SetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);

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
		LL_IWDG_ReloadCounter(IWDG);

		// Re-arm I2CS in case of lost master signal
		if (i2cs_state != I2CS_STATE_IDLE && ((uptime_ms() - i2cs_rearm_counter) > I2CS_REARM_TIMEOUT))
			i2cs_state = I2CS_STATE_IDLE;

		reg_sync();
		check_pmu_int();
		keyboard_process();
		hw_check_HP_presence();

		// Check internal status
		if (reg_get_value(REG_ID_SHTDW) == 1) {			// Nominal full system shutdown as requested from I2C bus
			reg_set_value(REG_ID_SHTDW, 0);
			LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);
			AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

			AXP2101_shutdown();
		} else if (reg_get_value(REG_ID_RST) == 1) {		// Try to reset only the STM32
			reg_set_value(REG_ID_RST, 0);
			HAL_Delay(200);		// Wait for final I2C answer
			if (HAL_I2C_DisableListen_IT(&hi2c1) != HAL_OK)
				Error_Handler();
			LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);

			NVIC_SystemReset();
		} else if (reg_get_value(REG_ID_RST_PICO) == 1) {		// Reset only the Pico
			reg_set_value(REG_ID_RST_PICO, 0);
			HAL_Delay(200);		// Wait for final I2C answer
			if (HAL_I2C_DisableListen_IT(&hi2c1) != HAL_OK)
				Error_Handler();
			LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);

			HAL_Delay(200);		// No need to use keyboard, so a simple delay should suffice
			LL_GPIO_SetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);
			LL_GPIO_SetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
				Error_Handler();
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



static void reset_to_bootloader(void) {
	// Disable IRQ to not break everything
	__disable_irq();

	// Reset the system to a boot state
	HAL_RCC_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	for (uint8_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}

	__enable_irq();

	// Relocate the main SP
	__set_MSP(STM32F1xxx_BL_VECTOR_TABLE->__sp);

	// Remap NVIC
	//SCB->VTOR = STM32F1xxx_BL_ADDR;

	// Wait for every datas to be ready
	__ISB();
	__DSB();

	// Now, jump to the bootloader!
	STM32F1xxx_BL_VECTOR_TABLE->__bl_call();

	for (;;)
		// We should never go here...
		continue;
}

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

		if (reg_is_bit_set(REG_ID_CFG, CFG_OVERFLOW_ON)) fifo_enqueue_force(item);
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
		//This is related to the battery charging and discharging logic. If you're not sure what you're doing, please don't modify it, as it could damage the battery.
		if (AXP2101_isDropWarningLevel1Irq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isDropWarningLevel1\n\r");
#endif
			//report_bat();
			//
			AXP2101_shutdown();
		}
		/*if (PMU.isGaugeWdtTimeoutIrq()) {
		  Serial1.println("isWdtTimeout");
		}
		if (PMU.isBatChargerOverTemperatureIrq()) {
		  Serial1.println("isBatChargeOverTemperature");
		}
		if (PMU.isBatWorkOverTemperatureIrq()) {
		  Serial1.println("isBatWorkOverTemperature");
		}
		if (PMU.isBatWorkUnderTemperatureIrq()) {
		  Serial1.println("isBatWorkUnderTemperature");
		}
		if (PMU.isVbusInsertIrq()) {
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
		if (AXP2101_isPekeyShortPressIrq()) {
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
			// enterPmuSleep();	//TODO: implement sleep mode, RTC, etc.?
		}
		if (AXP2101_isPekeyLongPressIrq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isPekeyLongPress\n\r");
#endif
			//Serial1.println("write pmu data buffer .");
			//uint8_t data[4] = {1, 2, 3, 4};
			//PMU.writeDataBuffer(data, XPOWERS_AXP2101_DATA_BUFFER_SIZE);

			reset_to_bootloader();

			/*LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);
			AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
			AXP2101_shutdown();*/
		}

		/*if (PMU.isPekeyNegativeIrq()) {
		  Serial1.println("isPekeyNegative");
		}
		if (PMU.isPekeyPositiveIrq()) {
		  Serial1.println("isPekeyPositive");
		}

		if (PMU.isLdoOverCurrentIrq()) {
		  Serial1.println("isLdoOverCurrentIrq");
		}
		if (PMU.isBatfetOverCurrentIrq()) {
		  Serial1.println("isBatfetOverCurrentIrq");
		}*/
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
		/*if (PMU.isBatDieOverTemperatureIrq()) {
		  Serial1.println("isBatDieOverTemperature");
		}
		if (PMU.isChagerOverTimeoutIrq()) {
		  Serial1.println("isChagerOverTimeout");
		}
		if (PMU.isBatOverVoltageIrq()) {
		  Serial1.println("isBatOverVoltage");
		}*/

		// Clear PMU Interrupt Status Register
		AXP2101_clearIrqStatus();
	}
}
