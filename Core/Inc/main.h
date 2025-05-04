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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_iwdg.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_ll_tim.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c2;

extern volatile uint32_t systicks_counter;
extern volatile uint8_t pmu_irq;
extern uint8_t io_matrix[9];
extern uint8_t js_bits;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
__STATIC_INLINE uint32_t uptime_ms(void) { return systicks_counter; }

void flash_one_time(uint32_t ts, uint8_t restore_status);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_LED_Pin LL_GPIO_PIN_13
#define SYS_LED_GPIO_Port GPIOC
#define COL_1_Pin LL_GPIO_PIN_0
#define COL_1_GPIO_Port GPIOC
#define COL_2_Pin LL_GPIO_PIN_1
#define COL_2_GPIO_Port GPIOC
#define COL_3_Pin LL_GPIO_PIN_2
#define COL_3_GPIO_Port GPIOC
#define COL_4_Pin LL_GPIO_PIN_3
#define COL_4_GPIO_Port GPIOC
#define ROW_1_Pin LL_GPIO_PIN_0
#define ROW_1_GPIO_Port GPIOA
#define ROW_2_Pin LL_GPIO_PIN_1
#define ROW_2_GPIO_Port GPIOA
#define ROW_3_Pin LL_GPIO_PIN_2
#define ROW_3_GPIO_Port GPIOA
#define ROW_4_Pin LL_GPIO_PIN_3
#define ROW_4_GPIO_Port GPIOA
#define ROW_5_Pin LL_GPIO_PIN_4
#define ROW_5_GPIO_Port GPIOA
#define ROW_6_Pin LL_GPIO_PIN_5
#define ROW_6_GPIO_Port GPIOA
#define ROW_7_Pin LL_GPIO_PIN_6
#define ROW_7_GPIO_Port GPIOA
#define ROW_8_Pin LL_GPIO_PIN_7
#define ROW_8_GPIO_Port GPIOA
#define COL_5_Pin LL_GPIO_PIN_4
#define COL_5_GPIO_Port GPIOC
#define COL_6_Pin LL_GPIO_PIN_5
#define COL_6_GPIO_Port GPIOC
#define KEY_1_Pin LL_GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOB
#define KEY_2_Pin LL_GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOB
#define KEY_3_Pin LL_GPIO_PIN_2
#define KEY_3_GPIO_Port GPIOB
#define PMU_SCL_Pin LL_GPIO_PIN_10
#define PMU_SCL_GPIO_Port GPIOB
#define PMU_SDA_Pin LL_GPIO_PIN_11
#define PMU_SDA_GPIO_Port GPIOB
#define KEY_9_Pin LL_GPIO_PIN_12
#define KEY_9_GPIO_Port GPIOB
#define KEY_10_Pin LL_GPIO_PIN_13
#define KEY_10_GPIO_Port GPIOB
#define KEY_11_Pin LL_GPIO_PIN_14
#define KEY_11_GPIO_Port GPIOB
#define KEY_12_Pin LL_GPIO_PIN_15
#define KEY_12_GPIO_Port GPIOB
#define COL_7_Pin LL_GPIO_PIN_6
#define COL_7_GPIO_Port GPIOC
#define COL_8_Pin LL_GPIO_PIN_7
#define COL_8_GPIO_Port GPIOC
#define KBD_BL_Pin LL_GPIO_PIN_8
#define KBD_BL_GPIO_Port GPIOC
#define PMU_IRQ_Pin LL_GPIO_PIN_9
#define PMU_IRQ_GPIO_Port GPIOC
#define PMU_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define LCD_BL_Pin LL_GPIO_PIN_8
#define LCD_BL_GPIO_Port GPIOA
#define PICO_EN_Pin LL_GPIO_PIN_13
#define PICO_EN_GPIO_Port GPIOA
#define SP_AMP_EN_Pin LL_GPIO_PIN_14
#define SP_AMP_EN_GPIO_Port GPIOA
#define PICO_UART_TX_Pin LL_GPIO_PIN_10
#define PICO_UART_TX_GPIO_Port GPIOC
#define PICO_UART_RX_Pin LL_GPIO_PIN_11
#define PICO_UART_RX_GPIO_Port GPIOC
#define HP_DET_Pin LL_GPIO_PIN_12
#define HP_DET_GPIO_Port GPIOC
#define KEY_4_Pin LL_GPIO_PIN_3
#define KEY_4_GPIO_Port GPIOB
#define KEY_5_Pin LL_GPIO_PIN_4
#define KEY_5_GPIO_Port GPIOB
#define KEY_6_Pin LL_GPIO_PIN_5
#define KEY_6_GPIO_Port GPIOB
#define KEY_7_Pin LL_GPIO_PIN_6
#define KEY_7_GPIO_Port GPIOB
#define KEY_8_Pin LL_GPIO_PIN_7
#define KEY_8_GPIO_Port GPIOB
#define PICO_SCL_Pin LL_GPIO_PIN_8
#define PICO_SCL_GPIO_Port GPIOB
#define PICO_SDA_Pin LL_GPIO_PIN_9
#define PICO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define EEPROM_VAR_ID	(0)		// 16b: Init ID: 0xCA1C
#define EEPROM_VAR_CFG	(1)		// 16b: 0x00 + CFG reg
#define EEPROM_VAR_KBD	(2)		// 16b: DEB + FRQ regs
#define EEPROM_VAR_BCKL	(3)		// 16b: LCD + KBD backlight step indice
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
