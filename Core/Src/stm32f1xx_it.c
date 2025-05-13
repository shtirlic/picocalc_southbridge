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
  */

#include "hal_interface.h"
#include "stm32f1xx_it.h"


extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
#ifdef DEBUG
extern UART_HandleTypeDef huart1;
#endif
#ifdef UART_PICO_INTERFACE
extern UART_HandleTypeDef huart3;
#endif


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {
	while (1) {}
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
	HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void) {
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);

		pmu_irq = 1;
	}
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void) {
	HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c1);
}

#ifdef DEBUG
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void) {
	HAL_UART_IRQHandler(&huart1);
}
#endif
#ifdef UART_PICO_INTERFACE
/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void) {
	HAL_UART_IRQHandler(&huart3);
}
#endif

/**
  * @brief This function handles RTC alarm interrupt through EXTI line 17.
  */
void RTC_Alarm_IRQHandler(void) {
	HAL_RTC_AlarmIRQHandler(&hrtc);
}
