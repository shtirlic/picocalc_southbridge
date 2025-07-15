#include "i2cs.h"

#include "hal_interface.h"
#include "backlight.h"
#include "fifo.h"
#include "rtc.h"
#include "regs.h"


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern RTC_HandleTypeDef hrtc;

static uint8_t i2cs_r_buff[5];
static volatile uint8_t i2cs_r_idx = 0;
static uint8_t i2cs_w_buff[10];
static volatile uint8_t i2cs_w_idx = 0;
static volatile uint8_t i2cs_w_len = 0;

enum i2cs_state i2cs_state = I2CS_STATE_IDLE;
uint32_t i2cs_rearm_counter = 0;


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
				} else if (reg == REG_ID_SYS_CFG) {
					if (is_write)
						reg_set_value(REG_ID_SYS_CFG, i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_SYS_CFG);
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
					if (is_write)
						reg_set_value(REG_ID_INT, 0);
					i2cs_w_buff[1] = reg_get_value(REG_ID_INT);
					LL_GPIO_SetOutputPin(PICO_IRQ_GPIO_Port, PICO_IRQ_Pin);	// De-assert the IRQ signal
				} else if (reg == REG_ID_VER) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_VER);
				} else if (reg == REG_ID_TYP) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_TYP);
				} else if (reg == REG_ID_BAT) {
					i2cs_w_buff[1] = reg_get_value(REG_ID_BAT);
				} else if (reg == REG_ID_RTC_CFG) {
					if (is_write) {
						rtc_reg_xor_events |= reg_get_value(REG_ID_RTC_CFG) ^ i2cs_r_buff[1];	// Using a "OR" set style to avoid loosing change before processing it
						reg_set_value(REG_ID_RTC_CFG, i2cs_r_buff[1]);
					}
					i2cs_w_buff[1] = reg_get_value(REG_ID_RTC_CFG);
				} else if (reg == REG_ID_RTC_DATE) {
					RTC_DateTypeDef date_s = {0};
					if (is_write) {
						i2cs_RTC_date_from_buffer(&date_s, &i2cs_r_buff[1]);

						HAL_RTC_SetDate(&hrtc, &date_s, RTC_FORMAT_BIN);
						force_date_bck_sync();
					}

					HAL_RTC_GetDate(&hrtc, &date_s, RTC_FORMAT_BIN);
					i2cs_fill_buffer_RTC_date(&i2cs_w_buff[1], &date_s);

					i2cs_w_len = 5;
				} else if (reg == REG_ID_RTC_TIME) {
					RTC_TimeTypeDef time_s = {0};
					if (is_write) {
						i2cs_RTC_time_from_buffer(&time_s, &i2cs_r_buff[1]);

						HAL_RTC_SetTime(&hrtc, &time_s, RTC_FORMAT_BIN);
					}

					HAL_RTC_GetTime(&hrtc, &time_s, RTC_FORMAT_BIN);
					i2cs_fill_buffer_RTC_time(&i2cs_w_buff[1], &time_s);

					i2cs_w_len = 4;
				} else if (reg == REG_ID_RTC_ALARM_DATE) {
					if (is_write)
						i2cs_RTC_date_from_buffer(&rtc_alarm_date._s, &i2cs_r_buff[1]);

					i2cs_fill_buffer_RTC_date(&i2cs_w_buff[1], &rtc_alarm_date._s);

					i2cs_w_len = 5;
				} else if (reg == REG_ID_RTC_ALARM_TIME) {
					if (is_write)
						i2cs_RTC_time_from_buffer(&rtc_alarm_time._s, &i2cs_r_buff[1]);

					i2cs_fill_buffer_RTC_time(&i2cs_w_buff[1], &rtc_alarm_time._s);

					i2cs_w_len = 4;
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
				} else if (reg == REG_ID_PWR_CTRL) {
					if (is_write)
						reg_set_value(REG_ID_PWR_CTRL, i2cs_r_buff[1]);
					i2cs_w_buff[1] = reg_get_value(REG_ID_PWR_CTRL);
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
				reg == REG_ID_SYS_CFG ||
				reg == REG_ID_INT_CFG ||
				reg == REG_ID_FRQ) {
				if (is_write)
					bytes_needed = 1;
			} else if (reg == REG_ID_DEB) {
				if (is_write)
					bytes_needed = 2;
			} else if (reg == REG_ID_RTC_DATE ||
					reg == REG_ID_RTC_ALARM_DATE ||
					reg == REG_ID_RTC_TIME ||
					reg == REG_ID_RTC_ALARM_TIME) {
				if (is_write)
					bytes_needed = 3;
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
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			//Error_Handler();
			// Actually this will trigger the watchdog and restart the system... That can ruin the day of the user.
			NVIC_SystemReset();
		}
}
