/**
  ******************************************************************************
  * @file         hal_interface.c
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

#include "hal_interface.h"
#include "stm32f1xx_hal_flash_ex.h"

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

WWDG_HandleTypeDef hwwdg;

#ifdef DEBUG
UART_HandleTypeDef huart1;
#endif
#ifdef UART_PICO_INTERFACE
UART_HandleTypeDef huart3;
#endif

volatile uint8_t rtc_reg_xor_events = 0;
volatile RTC_TimeTypeDef_u rtc_alarm_time = {.raw = 0x00000000};
volatile RTC_DateTypeDef_u rtc_alarm_date = {.raw = 0x00010101};

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0) {}
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while(LL_RCC_HSE_IsReady() != 1) {}
	LL_RCC_LSI_Enable();

	/* Wait till LSI is ready */
	while(LL_RCC_LSI_IsReady() != 1) {}
	LL_PWR_EnableBkUpAccess();
	if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE) {
		LL_RCC_ForceBackupDomainReset();
		LL_RCC_ReleaseBackupDomainReset();
	}
	LL_RCC_LSE_Enable();

	/* Wait till LSE is ready */
	while(LL_RCC_LSE_IsReady() != 1) {}
	if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE) {
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	}
	LL_RCC_EnableRTC();
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE) {}
	LL_SetSystemCoreClock(4000000);

	/* Update the time base */
	if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
		Error_Handler();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void) {
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
		Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
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
		Error_Handler();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {
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
		Error_Handler();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void) {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};

	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
		Error_Handler();

	/** Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		Error_Handler();

	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
		Error_Handler();
}

/**
  * @brief RTC fake initialization Function, used when RTC is already alive (after a wake-up reset)
  * @param None
  * @retval None
  */
static void MX_RTC_Init2(void) {
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	hrtc.Lock = HAL_UNLOCKED;
	HAL_RTC_MspInit(&hrtc);
	hrtc.State = HAL_RTC_STATE_READY;
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 800;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
		Error_Handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
		Error_Handler();

	HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		Error_Handler();
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 512;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		Error_Handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();

	HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {
#ifdef DEBUG
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		Error_Handler();
#endif
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void) {
#ifdef UART_PICO_INTERFACE
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
		Error_Handler();
#endif
}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void) {
	hwwdg.Instance = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_2;
	hwwdg.Init.Window = 127;
	hwwdg.Init.Counter = 127;
	hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
	if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
		Error_Handler();
	// Not using it for now
	__HAL_WWDG_DISABLE();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

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
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = COL_1_Pin|COL_2_Pin|COL_3_Pin|COL_4_Pin
						  |COL_5_Pin|COL_6_Pin|COL_7_Pin|COL_8_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#ifndef UART_PICO_INTERFACE
	GPIO_InitStruct.Pin = PICO_IRQ_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

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
}


/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void) {
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* System interrupt init*/

	/** DISABLE: JTAG-DP Disabled and SW-DP Disabled
	*/
	__HAL_AFIO_REMAP_SWJ_DISABLE();
}

/**
  * @brief CRC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc) {
	if(hcrc->Instance==CRC)
		__HAL_RCC_CRC_CLK_ENABLE();
}

/**
  * @brief CRC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc) {
	if(hcrc->Instance==CRC)
		__HAL_RCC_CRC_CLK_DISABLE();
}

/**
  * @brief I2C MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (hi2c->Instance == I2C1) {
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/**I2C1 GPIO Configuration
		PB8     ------> I2C1_SCL
		PB9     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_I2C1_ENABLE();

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();

		/* I2C1 interrupt Init */
		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	} else if (hi2c->Instance == I2C2) {
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
	}
}

/**
  * @brief I2C MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
	if (hi2c->Instance == I2C1) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		PB8     ------> I2C1_SCL
		PB9     ------> I2C1_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

		/* I2C1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
		HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
	} else if (hi2c->Instance == I2C2) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
	}
}

/**
  * @brief RTC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc) {
	if (hrtc->Instance == RTC) {
		HAL_PWR_EnableBkUpAccess();
		/* Enable BKP CLK enable for backup registers */
		__HAL_RCC_BKP_CLK_ENABLE();
		/* Peripheral clock enable */
		__HAL_RCC_RTC_ENABLE();
		/* RTC interrupt Init */
		HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	}
}

/**
  * @brief RTC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc) {
	if (hrtc->Instance == RTC) {
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE();

		/* RTC interrupt DeInit */
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
	}
}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
	if (htim_base->Instance == TIM1) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();
	} else if (htim_base->Instance == TIM2) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	} else if (htim_base->Instance == TIM3) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (htim->Instance == TIM1) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM1 GPIO Configuration
		PA8     ------> TIM1_CH1
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	} else if (htim->Instance == TIM3) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**TIM3 GPIO Configuration
		PC8     ------> TIM3_CH3
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_TIM3_ENABLE();
	}
}

/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
	if (htim_base->Instance == TIM1) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM1_CLK_DISABLE();
	} else if (htim_base->Instance == TIM2) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* TIM2 interrupt DeInit */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	} else if (htim_base->Instance == TIM3) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

#ifdef DEBUG
	if (huart->Instance == USART1) {
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART1 GPIO Configuration
		PA9     ------> USART1_TX
		PA10     ------> USART1_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	} else
#endif
#ifdef UART_PICO_INTERFACE
	if (huart->Instance == USART3) {
		/* Peripheral clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**USART3 GPIO Configuration
		PC10     ------> USART3_TX
		PC11     ------> USART3_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_USART3_PARTIAL();

		/* USART3 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}
#else
	NULL;
#endif
}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
#ifdef DEBUG
	if (huart->Instance == USART1) {
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		PA9     ------> USART1_TX
		PA10     ------> USART1_RX
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

		/* USART1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	} else
#endif
#ifdef UART_PICO_INTERFACE
	if (huart->Instance == USART3) {
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration
		PC10     ------> USART3_TX
		PC11     ------> USART3_RX
		*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

		/* USART3 interrupt DeInit */
		HAL_NVIC_DisableIRQ(USART3_IRQn);
	}
#else
	NULL;
#endif
}

/**
  * @brief WWDG MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hwwdg: WWDG handle pointer
  * @retval None
  */
void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg) {
	if(hwwdg->Instance==WWDG)
		__HAL_RCC_WWDG_CLK_ENABLE();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	// User can add his own implementation to report the HAL error return state
	//TODO: replace with dedicated, non-blocking, error handler
	__disable_irq();
	while (1) {
		LL_GPIO_SetOutputPin(SYS_LED_GPIO_Port, SYS_LED_Pin);
		HAL_Delay(500);
	}
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

void flash_one_time(uint32_t ts, uint8_t restore_status) {
	for (size_t i = 0; i < ts; i++) {
//#ifndef DEBUG
//		LL_IWDG_ReloadCounter(IWDG);
//#endif
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

/**
  * @brief  This function is executed at the start of the program to initialize all peripherals.
  * @retval None
  */
HAL_StatusTypeDef HAL_Interface_init(void) {
	HAL_StatusTypeDef result = HAL_OK;
	FLASH_OBProgramInitTypeDef flash_s;

	result |= HAL_Init();
	if (result != HAL_OK)
		return result;

	SystemClock_Config();

	// Control wake-up state
	//if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
	//__HAL_RCC_CLEAR_RESET_FLAGS();
	// Use SRAM backup registers to check if we have soft-reset
	HAL_PWR_EnableBkUpAccess();

	MX_CRC_Init();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0)
		MX_RTC_Init2();
	else
		MX_RTC_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
#ifndef DEBUG
	MX_WWDG_Init();
#endif
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();

	// Check options registers values
	HAL_FLASHEx_OBGetConfig(&flash_s);
	//if ((flash_s.USERConfig & (OB_STOP_NO_RST | OB_STDBY_NO_RST)) != 0) {
	if ((flash_s.USERConfig & (OB_STOP_NO_RST | OB_STDBY_NO_RST)) == 0) {
		HAL_FLASH_Unlock();
		HAL_FLASH_OB_Unlock();
		//flash_s.USERConfig &= (uint8_t)~(OB_STOP_NO_RST | OB_STDBY_NO_RST);	// Enable reset when sleep
		flash_s.USERConfig |= (uint8_t)(OB_STOP_NO_RST | OB_STDBY_NO_RST);	// Disable reset when sleep
		HAL_FLASHEx_OBProgram(&flash_s);
		HAL_FLASH_OB_Launch();	// Reset system

		// We should never reach this point
		for ( ;; ) {}
	}

	return result;
}
