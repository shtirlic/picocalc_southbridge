#include "axp2101.h"

#include "hal_interface.h"


#define AXP2101_DEV_I2C_ID	0x68


extern I2C_HandleTypeDef hi2c2;

static uint8_t statusRegister[XPOWERS_AXP2101_INTSTS_CNT] = {0};
static uint8_t intRegister[XPOWERS_AXP2101_INTSTS_CNT] = {0};


__STATIC_INLINE int8_t readRegister(uint8_t reg, uint8_t *buf, uint8_t length) {
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, reg, length, buf, 1, 60);
	if (status != HAL_OK)
		return -1;

	return 0;
}

__STATIC_INLINE uint16_t readRegisterH6L8(uint8_t highReg, uint8_t lowReg) {
	uint8_t h6;
	uint8_t l8;
	int8_t h6_s = readRegister(highReg, &h6, 1);
	int8_t l8_s = readRegister(lowReg, &l8, 1);
	if (h6_s == -1 || l8_s == -1)
		return 0;
	return ((h6 & 0x3F) << 8) | l8;
}

__STATIC_INLINE uint16_t readRegisterH5L8(uint8_t highReg, uint8_t lowReg) {
	uint8_t h5;
	uint8_t l8;
	int8_t h5_s = readRegister(highReg, &h5, 1);
	int8_t l8_s = readRegister(lowReg, &l8, 1);
	if (h5_s == -1 || l8_s == -1)
		return 0;
	return ((h5 & 0x1F) << 8) | l8;
}

__STATIC_INLINE uint8_t clrRegisterBit(uint8_t registers, uint8_t bit) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, registers, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	reg_value &= (uint8_t)(~_BV(bit));
	return HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, registers, 1, &reg_value, 1, 60);
}

__STATIC_INLINE uint8_t getRegisterBit(uint8_t registers, uint8_t bit) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, registers, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	return reg_value & _BV(bit);
}

__STATIC_INLINE uint8_t setRegisterBit(uint8_t registers, uint8_t bit) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, registers, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	reg_value |= (uint8_t)(_BV(bit));
	return HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, registers, 1, &reg_value, 1, 60);
}

uint32_t setInterruptImpl(uint32_t opts, uint8_t enable) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status = HAL_OK;

	if (opts & 0x0000FF) {
		HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTEN1, 1, &reg_value, 1, 60);
		intRegister[0] = enable ? (uint8_t)(reg_value | (opts & 0xFF)) : (uint8_t)(reg_value & (~(opts & 0xFF)));
		status |= HAL_I2C_Mem_Write(&hi2c2, 0x68, XPOWERS_AXP2101_INTEN1, 1, &intRegister[0], 1, 60);
	}
	if (opts & 0x00FF00) {
		HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTEN2, 1, &reg_value, 1, 60);
		intRegister[1] = enable ? (uint8_t)(reg_value | (opts >> 8)) : (uint8_t)(reg_value & (~(opts >> 8)));
		status |= HAL_I2C_Mem_Write(&hi2c2, 0x68, XPOWERS_AXP2101_INTEN2, 1, &intRegister[1], 1, 60);
	}
	if (opts & 0xFF0000) {
		HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTEN3, 1, &reg_value, 1, 60);
		intRegister[2] = enable ? (uint8_t)(reg_value | (opts >> 16)) : (uint8_t)(reg_value & (~(opts >> 16)));
		status |= HAL_I2C_Mem_Write(&hi2c2, 0x68, XPOWERS_AXP2101_INTEN3, 1, &intRegister[2], 1, 60);
	}

	return (uint32_t)status;
}


uint32_t AXP2101_shutdown(void) {
	return setRegisterBit(XPOWERS_AXP2101_COMMON_CONFIG, 0);
}

uint32_t AXP2101_disableTSPinMeasure(void) {
	return clrRegisterBit(XPOWERS_AXP2101_ADC_CHANNEL_CTRL, 1);
}

uint32_t AXP2101_enableBattDetection(void) {
	return setRegisterBit(XPOWERS_AXP2101_BAT_DET_CTRL, 0);
}

uint32_t AXP2101_enableBattVoltageMeasure(void) {
	return setRegisterBit(XPOWERS_AXP2101_ADC_CHANNEL_CTRL, 0);
}

uint32_t AXP2101_enableSystemVoltageMeasure(void) {
	return setRegisterBit(XPOWERS_AXP2101_ADC_CHANNEL_CTRL, 3);
}

uint32_t AXP2101_enableVbusVoltageMeasure(void) {
	return setRegisterBit(XPOWERS_AXP2101_ADC_CHANNEL_CTRL, 2);
}

uint32_t AXP2101_enableIRQ(uint32_t opt) {
	return setInterruptImpl(opt, 1);
}

uint32_t AXP2101_disableIRQ(uint32_t opt) {
	return setInterruptImpl(opt, 0);
}

uint32_t AXP2101_clearIrqStatus(void) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t fbuff = 0xFF;

	for (size_t i = 0; i < XPOWERS_AXP2101_INTSTS_CNT; i++) {
		status |= HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTSTS1 + (uint8_t)i, 1, &fbuff, 1, 60);
		statusRegister[i] = 0;
	}

	return (uint32_t)status;
}

uint8_t AXP2101_isDropWarningLevel1Irq(void) {
	uint8_t mask = XPOWERS_AXP2101_WARNING_LEVEL1_IRQ;
	if (intRegister[0] & mask)
		return ((statusRegister[0] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isVbusRemoveIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_VBUS_REMOVE_IRQ >> 8;
	if (intRegister[1] & mask)
		return ((statusRegister[1] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isBatInsertIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_BAT_INSERT_IRQ >> 8;
	if (intRegister[1] & mask)
		return ((statusRegister[1] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isBatRemoveIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_BAT_REMOVE_IRQ >> 8;
	if (intRegister[1] & mask)
		return ((statusRegister[1] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isPekeyShortPressIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_PKEY_SHORT_IRQ >> 8;
	if (intRegister[1] & mask)
		return ((statusRegister[1] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isPekeyLongPressIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_PKEY_LONG_IRQ >> 8;
	if (intRegister[1] & mask)
		return ((statusRegister[1] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isBatChargeDoneIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_BAT_CHG_DONE_IRQ >> 16;
	if (intRegister[2] & mask)
		return ((statusRegister[2] & mask) == mask);
	return 0;
}

uint8_t AXP2101_isBatChargeStartIrq(void) {
	uint8_t mask = XPOWERS_AXP2101_BAT_CHG_START_IRQ >> 16;
	if (intRegister[2] & mask)
		return ((statusRegister[2] & mask) == mask);
	return 0;
}


// value in mV
uint32_t AXP2101_setSysPowerDownVoltage(uint16_t value) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	if (value % XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS) {
		//log_e("Mistake ! The steps is must %u mV", XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS);
		return 10;
	}
	if (value < XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN) {
		//log_e("Mistake ! The minimum settable voltage of VSYS is %u mV", XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN);
		return 10;
	} else if (value > XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MAX) {
		//log_e("Mistake ! The maximum settable voltage of VSYS is %u mV", XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MAX);
		return 10;
	}

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_VOFF_SET, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	reg_value &= 0xF8;
	reg_value |= (uint8_t)((value - XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN) / XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS);
	return (uint32_t)HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_VOFF_SET, 1, &reg_value, 1, 60);
}

uint32_t AXP2101_setChargingLedMode(uint8_t mode) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	switch (mode) {
	case XPOWERS_CHG_LED_OFF:
	// clrRegisterBit(XPOWERS_AXP2101_CHGLED_SET_CTRL, 0);
	// break;
	case XPOWERS_CHG_LED_BLINK_1HZ:
	case XPOWERS_CHG_LED_BLINK_4HZ:
	case XPOWERS_CHG_LED_ON:
		status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_CHGLED_SET_CTRL, 1, &reg_value, 1, 60);
		if (status != HAL_OK)
			return 1;

		reg_value &= 0xC8;
		reg_value |= 0x05;    //use manual ctrl
		reg_value |= (mode << 4);

		status = HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_CHGLED_SET_CTRL, 1, &reg_value, 1, 60);
		break;
	case XPOWERS_CHG_LED_CTRL_CHG:
		status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_CHGLED_SET_CTRL, 1, &reg_value, 1, 60);
		if (status != HAL_OK)
			return 1;

		reg_value &= 0xF9;
		reg_value |= 0x01;	// use type A mode
		//reg_value |= 0x02;	// use type B mode

		status = HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_CHGLED_SET_CTRL, 1, &reg_value, 1, 60);
		break;
	default:
		status = 10;
		break;
	}

	return (uint32_t)status;
}

/**
 * @brief  Low battery warning threshold 5-20%, 1% per step
 * @param  percentage:   5 ~ 20
 * @retval Status code
 */
uint32_t AXP2101_setLowBatWarnThreshold(uint8_t percentage) {
	if (percentage < 5 || percentage > 20)
		return 1;

	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_LOW_BAT_WARN_SET, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	reg_value &= 0x0F;
	reg_value |= (uint8_t)((percentage - 5) << 4);
	return HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_LOW_BAT_WARN_SET, 1, &reg_value, 1, 60);
}

/**
 * @brief  Low battery shutdown threshold 0-15%, 1% per step
 * @param  opt:   0 ~ 15
 * @retval Status code
 */
uint32_t AXP2101_setLowBatShutdownThreshold(uint8_t opt) {
	uint8_t reg_value = 0;
	HAL_StatusTypeDef status;

	if (opt > 15)
		opt = 15;

	status = HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_LOW_BAT_WARN_SET, 1, &reg_value, 1, 60);
	if (status != HAL_OK)
		return 1;

	reg_value &= 0xF0;
	reg_value |= opt;
	return HAL_I2C_Mem_Write(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_LOW_BAT_WARN_SET, 1, &reg_value, 1, 60);
}


int8_t AXP2101_readDataBuffer(uint8_t *data, uint8_t size) {
	if (size > XPOWERS_AXP2101_DATA_BUFFER_SIZE)
		return -1;
	return readRegister(XPOWERS_AXP2101_DATA_BUFFER1, data, size);
}

uint8_t AXP2101_isBatteryConnect(void) {
    return getRegisterBit(XPOWERS_AXP2101_STATUS1, 3);
}

uint8_t AXP2101_isCharging(void) {
	uint8_t reg_value = 0;

	HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_STATUS2, 1, &reg_value, 1, 60);

	return (reg_value >> 5) == 0x01;
}

uint8_t AXP2101_isDischarge(void) {
	uint8_t res = 0;
	readRegister(XPOWERS_AXP2101_STATUS2, &res, 1);
	return (res >> 5) == 0x02;
}

uint8_t AXP2101_isStandby(void) {
	uint8_t res = 0;
	readRegister(XPOWERS_AXP2101_STATUS2, &res, 1);
	return (res >> 5) == 0x00;
}

uint8_t AXP2101_isVbusGood(void) {
	return getRegisterBit(XPOWERS_AXP2101_STATUS1, 5);
}

uint8_t AXP2101_isVbusIn(void) {
	return getRegisterBit(XPOWERS_AXP2101_STATUS2, 3) == 0 && AXP2101_isVbusGood();
}

uint32_t AXP2101_getIrqStatus(uint32_t* out_value) {
	HAL_StatusTypeDef status = HAL_OK;

	status |= HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTSTS1, 1, &statusRegister[0], 1, 60);
	status |= HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTSTS2, 1, &statusRegister[1], 1, 60);
	status |= HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_INTSTS3, 1, &statusRegister[2], 1, 60);

	((uint8_t*)&out_value)[0] = statusRegister[0];
	((uint8_t*)&out_value)[0] = statusRegister[1];
	((uint8_t*)&out_value)[0] = statusRegister[2];
	((uint8_t*)&out_value)[3] = 0xFF;

	return (uint32_t)status;
}

uint32_t AXP2101_getBatteryPercent(uint8_t* out_value) {
	if (!AXP2101_isBatteryConnect())
		return 1;

	return HAL_I2C_Mem_Read(&hi2c2, AXP2101_DEV_I2C_ID, XPOWERS_AXP2101_BAT_PERCENT_DATA, 1, out_value, 1, 60);
}

xpowers_chg_status_t AXP2101_getChargerStatus(void) {
	uint8_t val = 0;
	int8_t status = readRegister(XPOWERS_AXP2101_STATUS2, &val, 1);
	if (status == -1)
		return XPOWERS_AXP2101_CHG_STOP_STATE;
	val &= 0x07;
	return (xpowers_chg_status_t)val;
}

uint16_t AXP2101_getBattVoltage(void) {
	if (!AXP2101_isBatteryConnect()) {
		return 0;
	}
	return readRegisterH5L8(XPOWERS_AXP2101_ADC_DATA_RELUST0, XPOWERS_AXP2101_ADC_DATA_RELUST1);
}

uint16_t AXP2101_getVbusVoltage(void) {
	if (!AXP2101_isVbusIn()) {
		return 0;
	}
	return readRegisterH6L8(XPOWERS_AXP2101_ADC_DATA_RELUST4, XPOWERS_AXP2101_ADC_DATA_RELUST5);
}

uint16_t AXP2101_getSystemVoltage(void) {
	return readRegisterH6L8(XPOWERS_AXP2101_ADC_DATA_RELUST6, XPOWERS_AXP2101_ADC_DATA_RELUST7);
}
