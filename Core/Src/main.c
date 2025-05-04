/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "axp2101.h"
#include "backlight.h"
#include "batt.h"
#include "eeprom.h"
#include "fifo.h"
#include "keyboard.h"
#include "regs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum i2cs_state {
	//I2CS_STATE_HALT,
	I2CS_STATE_IDLE,
	I2CS_STATE_REG_REQUEST,
	I2CS_STATE_REG_ANSWER
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DEFAULT_LCD_BL	(205)	// ~40% PWM@7.81kHz (9 bits resolution)
//#define DEFAULT_KBD_BL	(20)	// ~4% PWM@7.81kHz (9 bits resolution)
#define DEFAULT_LCD_BL	(3)		//step-4 (~50%)
#define DEFAULT_KBD_BL	(0)		//step-1 (0%)

#define I2CS_REARM_TIMEOUT	500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef DEBUG
#define DEBUG_UART_MSG(msg)			HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 1000)
//#define DEBUG_UART_MSG2(d,s)		HAL_UART_Transmit(&huart1, (uint8_t*)d, s, 200)
#define DEBUG_UART_MSG2(d,sz, swp)	uart_rawdata_write(d,sz,swp)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint32_t systicks_counter = 0;	// 1 MHz systick counter - TODO: implement overflow self-reset mechanism
volatile uint32_t pmu_check_counter = 0;
volatile uint32_t i2cs_rearm_counter = 0;
#ifdef DEBUG
static const uint8_t hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
#endif

static uint8_t i2cs_r_buff[2];
static volatile uint8_t i2cs_r_idx = 0;
static uint8_t i2cs_w_buff[31 + 1];		// The last one must be only a 0 value
static volatile uint8_t i2cs_w_idx = 0;
static volatile uint8_t i2cs_w_len = 0;
static enum i2cs_state i2cs_state = I2CS_STATE_IDLE;

static uint8_t keycb_start = 0;
static uint32_t head_phone_status = 0;
volatile uint8_t pmu_irq = 0;
static uint32_t pmu_online = 0;

uint8_t io_matrix[9] = {0};		//for IO matrix,last byte is the restore key(c64 only)
uint8_t js_bits = 0xFF;			// c64 joystick bits
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//static void lock_cb(uint8_t caps_changed, uint8_t num_changed);
static void key_cb(char key, enum key_state state);
static void hw_check_HP_presence(void);
static void sync_bat(void);
static void check_pmu_int(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		systicks_counter += 1;
		i2cs_rearm_counter += 1;
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

				i2cs_rearm_counter = 0;
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
				} else if (reg == REG_ID_FIF) {
					struct fifo_item item = {0};
					fifo_dequeue(&item);
					i2cs_w_buff[0] = item.state;
					i2cs_w_buff[1] = item.key;
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
				} else {
					i2cs_w_buff[0] = 0;
					i2cs_w_buff[1] = 0;
				}

				i2cs_state = I2CS_STATE_REG_ANSWER;
				i2cs_w_idx = 0;

				HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, i2cs_w_buff, i2cs_w_len, I2C_FIRST_AND_LAST_FRAME);

				i2cs_rearm_counter = 0;
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

			if (reg == REG_ID_BKL) {	// We wait an another byte for these registers
				if (is_write) {
					HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2cs_r_buff + i2cs_r_idx, 1, I2C_NEXT_FRAME);	// This write the second received byte to i2cs_r_buff[1]
				}
			} else if (reg == REG_ID_BK2) {
				if (is_write) {
					HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2cs_r_buff + i2cs_r_idx, 1, I2C_NEXT_FRAME);	// This write the second received byte to i2cs_r_buff[1]
				}
			}
		}
	}
}

/*extern void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		if (i2cs_state == I2CS_STATE_REG_ANSWER) {
			if (++i2cs_w_idx < i2cs_w_len) {
				HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, i2cs_w_buff + i2cs_w_idx, 1, I2C_NEXT_FRAME);	// This write the next answer byte on I2C bus
			} else {
				i2cs_w_buff[31] = 0;
				HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2cs_w_buff[31], 1, I2C_NEXT_FRAME);	// send a 0 value to avoid stalling - TODO: usefull? can we use I2C_LAST_FRAME instead?
			}

			i2cs_rearm_counter = 0;
		}
	}
}*/

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
			Error_Handler();	//TODO: replace with dedicated, non-blocking, error handler
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

void flash_one_time(uint32_t ts, uint8_t restore_status) {
	for (size_t i = 0; i < ts; i++) {
		LL_IWDG_ReloadCounter(IWDG);
		LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
		HAL_Delay(400);
		LL_GPIO_SetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
		HAL_Delay(200);
	}

	if (restore_status)
		LL_GPIO_ResetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
	else
		LL_GPIO_SetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int32_t result = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

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
	  EEPROM_WriteVariable(EEPROM_VAR_KBD, (EEPROM_Value)(uint16_t)((10 << 8) | 5), EEPROM_SIZE16);
	  EEPROM_WriteVariable(EEPROM_VAR_CFG, (EEPROM_Value)(uint16_t)(CFG_OVERFLOW_INT | CFG_KEY_INT | CFG_USE_MODS | CFG_REPORT_MODS), EEPROM_SIZE16);
	  EEPROM_WriteVariable(EEPROM_VAR_ID, (EEPROM_Value)(uint16_t)0xCA1C, EEPROM_SIZE16);
#ifdef DEBUG
	  DEBUG_UART_MSG("EEPROM first start!\n\r");
#endif
  }

  // I2C-Pico interface registers
  reg_init();
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  	  Error_Handler();
  HAL_Delay(10);

  // Check for AXP2101 is accessible on secondary I2C bus
  //result = HAL_I2C_IsDeviceReady(&hi2c2, 0x68, 3, 40);
  //if (result == HAL_OK) {
  result = 0;
  HAL_I2C_Mem_Read(&hi2c2, 0x68, XPOWERS_AXP2101_IC_TYPE, 1, (uint8_t*)&result, 1, 60);
  if (result == XPOWERS_AXP2101_CHIP_ID) {
#ifdef DEBUG
		DEBUG_UART_MSG("PMU ID: ");
		DEBUG_UART_MSG2((uint32_t)result, 1, 0);
		DEBUG_UART_MSG("\n\r");
#endif
		pmu_online = 1;
  } else {
#ifdef DEBUG
	  DEBUG_UART_MSG("PMU not online!\n\r");
#endif
  }

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

  HAL_Delay(1000);
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
					// XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ |
					// XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	LL_IWDG_ReloadCounter(IWDG);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Save user registers in EEPROM if unsynced every 2.5s
	reg_sync();

	// Re-arm I2CS in case of lost master signal
	if (i2cs_state != I2CS_STATE_IDLE && i2cs_rearm_counter > I2CS_REARM_TIMEOUT)
		i2cs_state = I2CS_STATE_IDLE;

	check_pmu_int();
	keyboard_process();
	hw_check_HP_presence();
	//HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
  }
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  }
  LL_RCC_EnableRTC();
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {

  }
  LL_SetSystemCoreClock(4000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 62;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
#ifndef DEBUG
  /* USER CODE END IWDG_Init 1 */
  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);
  LL_IWDG_SetReloadCounter(IWDG, 4095);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }

  LL_IWDG_ReloadCounter(IWDG);
  /* USER CODE BEGIN IWDG_Init 2 */
#endif
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};

    LL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_BKP);
  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC and set the Time and Date
  */
  RTC_InitStruct.AsynchPrescaler = 0xFFFFFFFFU;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_SetAsynchPrescaler(RTC, 0xFFFFFFFFU);

  /** Initialize RTC and set the Time and Date
  */
  RTC_TimeStruct.Hours = 0;
  RTC_TimeStruct.Minutes = 0;
  RTC_TimeStruct.Seconds = 0;
  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);

  /** Initialize RTC and set the Time and Date
  */
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 512;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
#ifdef DEBUG
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
#endif
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
#ifdef UART_PICO_INTERFACE
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
#endif
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, SYS_LED_Pin|COL_1_Pin|COL_2_Pin|COL_3_Pin
                          |COL_4_Pin|COL_5_Pin|COL_6_Pin|COL_7_Pin
                          |COL_8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, PICO_EN_Pin|SP_AMP_EN_Pin);

  /**/
  GPIO_InitStruct.Pin = SYS_LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = COL_1_Pin|COL_2_Pin|COL_3_Pin|COL_4_Pin
                          |COL_5_Pin|COL_6_Pin|COL_7_Pin|COL_8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ROW_1_Pin|ROW_2_Pin|ROW_3_Pin|ROW_4_Pin
                          |ROW_5_Pin|ROW_6_Pin|ROW_7_Pin|ROW_8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_2_Pin|KEY_3_Pin|KEY_9_Pin
                          |KEY_10_Pin|KEY_11_Pin|KEY_12_Pin|KEY_4_Pin
                          |KEY_5_Pin|KEY_6_Pin|KEY_7_Pin|KEY_8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PICO_EN_Pin|SP_AMP_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = HP_DET_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(HP_DET_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE9);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(PMU_IRQ_GPIO_Port, PMU_IRQ_Pin, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
static void lock_cb(uint8_t caps_changed, uint8_t num_changed) {
	//uint8_t do_int = 0;

	if (caps_changed && reg_is_bit_set(REG_ID_CFG, CFG_CAPSLOCK_INT)) {
		reg_set_bit(REG_ID_INT, INT_CAPSLOCK);
		//do_int = 1;
	}

	if (num_changed && reg_is_bit_set(REG_ID_CFG, CFG_NUMLOCK_INT)) {
		reg_set_bit(REG_ID_INT, INT_NUMLOCK);
		//do_int = 1;
	}

	// int_pin can be a LED
	if (do_int) {
		port_pin_set_output_level(int_pin, 0);
		delay_ms(INT_DURATION_MS);
		port_pin_set_output_level(int_pin, 1);
	}
}
*/

static void key_cb(char key, enum key_state state) {
	if (keycb_start == 0) {
		fifo_flush();
		return;
	}

	if (reg_is_bit_set(REG_ID_CFG, CFG_KEY_INT)) {
		reg_set_bit(REG_ID_INT, INT_KEY);
	}

#ifdef DEBUG
	// Serial1.println("key: 0x%02X/%d/%c, state: %d, blk: %d\r\n", key, key, key, state, reg_get_value(REG_ID_BKL));
	//HAL_UART_Transmit_IT(&huart1, HP_PLUG_MSG, HP_PLUG_MSG_LEN);
#endif

	const struct fifo_item item = {key, state};
	if (!fifo_enqueue(item)) {
		if (reg_is_bit_set(REG_ID_CFG, CFG_OVERFLOW_INT)) {
			reg_set_bit(REG_ID_INT, INT_OVERFLOW);  // INT_OVERFLOW  The interrupt was generated by FIFO overflow.
		}

		if (reg_is_bit_set(REG_ID_CFG, CFG_OVERFLOW_ON)) fifo_enqueue_force(item);
	}

#ifdef DEBUG
	//Serial1.println(key);
	//HAL_UART_Transmit_IT(&huart1, HP_PLUG_MSG, HP_PLUG_MSG_LEN);
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

		/*if (AXP2101_isPekeyShortPressIrq()) {
		  Serial1.println("isPekeyShortPress");
		  // enterPmuSleep();

		  Serial1.print("Read pmu data buffer .");
		  uint8_t data[4] = {0};
		  PMU.readDataBuffer(data, XPOWERS_AXP2101_DATA_BUFFER_SIZE);
		  for (int i = 0; i < 4; ++i) {
			Serial1.print(data[i]);
			Serial1.print(",");
		  }
		  Serial1.println();

		  printPMU();
		}*/

		if (AXP2101_isPekeyLongPressIrq()) {
#ifdef DEBUG
			DEBUG_UART_MSG("PMU: isPekeyLongPress\n\r");
#endif
			//Serial1.println("write pmu data buffer .");
			//uint8_t data[4] = {1, 2, 3, 4};
			//PMU.writeDataBuffer(data, XPOWERS_AXP2101_DATA_BUFFER_SIZE);

			LL_GPIO_ResetOutputPin(SP_AMP_EN_GPIO_Port, SP_AMP_EN_Pin);
			LL_GPIO_ResetOutputPin(PICO_EN_GPIO_Port, PICO_EN_Pin);
			AXP2101_setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
			AXP2101_shutdown();
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
	  //LL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
	  HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
