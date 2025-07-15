/**
  ******************************************************************************
  * @file         hal_interface.h
  * @brief        Central access to STM32-HAL definitions and
  *               related functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 C.ARE (JackCarterSmith).
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  *
  */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_crc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_wwdg.h"


#ifndef HAL_INTERFACE_H_
#define HAL_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

// HAL GPIO pins definition --------------------------------------------------------
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
#ifdef UART_PICO_INTERFACE
#define PICO_UART_TX_Pin LL_GPIO_PIN_10
#define PICO_UART_TX_GPIO_Port GPIOC
#define PICO_UART_RX_Pin LL_GPIO_PIN_11
#define PICO_UART_RX_GPIO_Port GPIOC
#else
#define PICO_IRQ_Pin LL_GPIO_PIN_10
#define PICO_IRQ_GPIO_Port GPIOC
#endif
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

#define LCD_BCKL_STEPS 10
#define KBD_BCKL_STEPS 4


#ifdef __HAL_WWDG_ENABLE
#undef __HAL_WWDG_ENABLE
#endif
#define __HAL_WWDG_ENABLE()	__HAL_RCC_WWDG_CLK_ENABLE()

#ifdef __HAL_WWDG_DISABLE
#undef __HAL_WWDG_DISABLE
#endif
#define __HAL_WWDG_DISABLE()	__HAL_RCC_WWDG_CLK_DISABLE()


// Structure definition ---------------------------------------------------------------
typedef union {
    uint32_t raw;
    RTC_TimeTypeDef _s;
} RTC_TimeTypeDef_u;

typedef union {
    uint32_t raw;
    RTC_DateTypeDef _s;
} RTC_DateTypeDef_u;


// Global variables definition --------------------------------------------------------
extern volatile uint32_t systicks_counter;
extern volatile uint8_t pmu_irq;
extern volatile uint8_t stop_mode_active;

extern volatile uint8_t rtc_reg_xor_events;
extern volatile RTC_TimeTypeDef_u rtc_alarm_time;
extern volatile RTC_DateTypeDef_u rtc_alarm_date;


// Global functions definition --------------------------------------------------------
void SystemClock_Config(void);
HAL_StatusTypeDef HAL_Interface_init(void);

__STATIC_INLINE uint32_t uptime_ms(void) { return systicks_counter; }
void flash_one_time(uint32_t ts, uint8_t restore_status);

void Error_Handler(void);

#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACE_H_ */
