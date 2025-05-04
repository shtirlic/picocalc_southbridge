#include "stm32f1xx_hal.h"


#ifndef AXP2101_H_
#define AXP2101_H_

/*
 * --------------------- I2C registers ------------------------
 */
#define XPOWERS_AXP2101_CHIP_ID                          (0x4A)

#define XPOWERS_AXP2101_STATUS1                          (0x00)
#define XPOWERS_AXP2101_STATUS2                          (0x01)
#define XPOWERS_AXP2101_IC_TYPE                          (0x03)


//#define XPOWERS_AXP2101_DATA_BUFFER1                     (0x04)
//#define XPOWERS_AXP2101_DATA_BUFFER2                     (0x05)
//#define XPOWERS_AXP2101_DATA_BUFFER3                     (0x06)
//#define XPOWERS_AXP2101_DATA_BUFFER4                     (0x07)
//#define XPOWERS_AXP2101_DATA_BUFFER_SIZE                 (4u)

#define XPOWERS_AXP2101_COMMON_CONFIG                    (0x10)
//#define XPOWERS_AXP2101_BATFET_CTRL                      (0x12)
//#define XPOWERS_AXP2101_DIE_TEMP_CTRL                    (0x13)
//#define XPOWERS_AXP2101_MIN_SYS_VOL_CTRL                 (0x14)
//#define XPOWERS_AXP2101_INPUT_VOL_LIMIT_CTRL             (0x15)
//#define XPOWERS_AXP2101_INPUT_CUR_LIMIT_CTRL             (0x16)
//#define XPOWERS_AXP2101_RESET_FUEL_GAUGE                 (0x17)
//#define XPOWERS_AXP2101_CHARGE_GAUGE_WDT_CTRL            (0x18)


//#define XPOWERS_AXP2101_WDT_CTRL                         (0x19)
#define XPOWERS_AXP2101_LOW_BAT_WARN_SET                 (0x1A)


//#define XPOWERS_AXP2101_PWRON_STATUS                     (0x20)
//#define XPOWERS_AXP2101_PWROFF_STATUS                    (0x21)
//#define XPOWERS_AXP2101_PWROFF_EN                        (0x22)
//#define XPOWERS_AXP2101_DC_OVP_UVP_CTRL                  (0x23)
#define XPOWERS_AXP2101_VOFF_SET                         (0x24)
//#define XPOWERS_AXP2101_PWROK_SEQU_CTRL                  (0x25)
//#define XPOWERS_AXP2101_SLEEP_WAKEUP_CTRL                (0x26)
//#define XPOWERS_AXP2101_IRQ_OFF_ON_LEVEL_CTRL            (0x27)

//#define XPOWERS_AXP2101_FAST_PWRON_SET0                  (0x28)
//#define XPOWERS_AXP2101_FAST_PWRON_SET1                  (0x29)
//#define XPOWERS_AXP2101_FAST_PWRON_SET2                  (0x2A)
//#define XPOWERS_AXP2101_FAST_PWRON_CTRL                  (0x2B)

#define XPOWERS_AXP2101_ADC_CHANNEL_CTRL                 (0x30)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST0                 (0x34)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST1                 (0x35)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST2                 (0x36)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST3                 (0x37)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST4                 (0x38)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST5                 (0x39)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST6                 (0x3A)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST7                 (0x3B)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST8                 (0x3C)
//#define XPOWERS_AXP2101_ADC_DATA_RELUST9                 (0x3D)

/*
 * ------------------ Interrupt registers ---------------------
 */
#define XPOWERS_AXP2101_INTEN1                           (0x40)
#define XPOWERS_AXP2101_INTEN2                           (0x41)
#define XPOWERS_AXP2101_INTEN3                           (0x42)

/*
 * -------------------- Status registers -----------------------
 */
#define XPOWERS_AXP2101_INTSTS1                          (0x48)
#define XPOWERS_AXP2101_INTSTS2                          (0x49)
#define XPOWERS_AXP2101_INTSTS3                          (0x4A)
#define XPOWERS_AXP2101_INTSTS_CNT                       (3)

#define XPOWERS_AXP2101_TS_PIN_CTRL                      (0x50)
#define XPOWERS_AXP2101_TS_HYSL2H_SET                    (0x52)
#define XPOWERS_AXP2101_TS_LYSL2H_SET                    (0x53)


#define XPOWERS_AXP2101_VLTF_CHG_SET                     (0x54)
#define XPOWERS_AXP2101_VHLTF_CHG_SET                    (0x55)
#define XPOWERS_AXP2101_VLTF_WORK_SET                    (0x56)
#define XPOWERS_AXP2101_VHLTF_WORK_SET                   (0x57)


#define XPOWERS_AXP2101_JIETA_EN_CTRL                    (0x58)
#define XPOWERS_AXP2101_JIETA_SET0                       (0x59)
#define XPOWERS_AXP2101_JIETA_SET1                       (0x5A)
#define XPOWERS_AXP2101_JIETA_SET2                       (0x5B)


#define XPOWERS_AXP2101_IPRECHG_SET                      (0x61)
#define XPOWERS_AXP2101_ICC_CHG_SET                      (0x62)
#define XPOWERS_AXP2101_ITERM_CHG_SET_CTRL               (0x63)

#define XPOWERS_AXP2101_CV_CHG_VOL_SET                   (0x64)

#define XPOWERS_AXP2101_THE_REGU_THRES_SET               (0x65)
#define XPOWERS_AXP2101_CHG_TIMEOUT_SET_CTRL             (0x67)

#define XPOWERS_AXP2101_BAT_DET_CTRL                     (0x68)
#define XPOWERS_AXP2101_CHGLED_SET_CTRL                  (0x69)

#define XPOWERS_AXP2101_BTN_VOL_MIN                      (2600)
#define XPOWERS_AXP2101_BTN_VOL_MAX                      (3300)
#define XPOWERS_AXP2101_BTN_VOL_STEPS                    (100)


#define XPOWERS_AXP2101_BTN_BAT_CHG_VOL_SET              (0x6A)


#define XPOWERS_AXP2101_DC_ONOFF_DVM_CTRL                (0x80)
#define XPOWERS_AXP2101_DC_FORCE_PWM_CTRL                (0x81)
#define XPOWERS_AXP2101_DC_VOL0_CTRL                     (0x82)
#define XPOWERS_AXP2101_DC_VOL1_CTRL                     (0x83)
#define XPOWERS_AXP2101_DC_VOL2_CTRL                     (0x84)
#define XPOWERS_AXP2101_DC_VOL3_CTRL                     (0x85)
#define XPOWERS_AXP2101_DC_VOL4_CTRL                     (0x86)


#define XPOWERS_AXP2101_LDO_ONOFF_CTRL0                  (0x90)
#define XPOWERS_AXP2101_LDO_ONOFF_CTRL1                  (0x91)
#define XPOWERS_AXP2101_LDO_VOL0_CTRL                    (0x92)
#define XPOWERS_AXP2101_LDO_VOL1_CTRL                    (0x93)
#define XPOWERS_AXP2101_LDO_VOL2_CTRL                    (0x94)
#define XPOWERS_AXP2101_LDO_VOL3_CTRL                    (0x95)
#define XPOWERS_AXP2101_LDO_VOL4_CTRL                    (0x96)
#define XPOWERS_AXP2101_LDO_VOL5_CTRL                    (0x97)
#define XPOWERS_AXP2101_LDO_VOL6_CTRL                    (0x98)
#define XPOWERS_AXP2101_LDO_VOL7_CTRL                    (0x99)
#define XPOWERS_AXP2101_LDO_VOL8_CTRL                    (0x9A)


#define XPOWERS_AXP2101_BAT_PARAME                       (0xA1)
#define XPOWERS_AXP2101_FUEL_GAUGE_CTRL                  (0xA2)
#define XPOWERS_AXP2101_BAT_PERCENT_DATA                 (0xA4)

/*
 * ------------------------ DCDC 1~5 ---------------------------
 */
#define XPOWERS_AXP2101_DCDC1_VOL_MIN                    (1500)
#define XPOWERS_AXP2101_DCDC1_VOL_MAX                    (3400)
#define XPOWERS_AXP2101_DCDC1_VOL_STEPS                  (100u)

#define XPOWERS_AXP2101_DCDC2_VOL1_MIN                  (500u)
#define XPOWERS_AXP2101_DCDC2_VOL1_MAX                  (1200u)
#define XPOWERS_AXP2101_DCDC2_VOL2_MIN                  (1220u)
#define XPOWERS_AXP2101_DCDC2_VOL2_MAX                  (1540u)

#define XPOWERS_AXP2101_DCDC2_VOL_STEPS1                 (10u)
#define XPOWERS_AXP2101_DCDC2_VOL_STEPS2                 (20u)

#define XPOWERS_AXP2101_DCDC2_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP2101_DCDC2_VOL_STEPS2_BASE            (71)


#define XPOWERS_AXP2101_DCDC3_VOL1_MIN                  (500u)
#define XPOWERS_AXP2101_DCDC3_VOL1_MAX                  (1200u)
#define XPOWERS_AXP2101_DCDC3_VOL2_MIN                  (1220u)
#define XPOWERS_AXP2101_DCDC3_VOL2_MAX                  (1540u)
#define XPOWERS_AXP2101_DCDC3_VOL3_MIN                  (1600u)
#define XPOWERS_AXP2101_DCDC3_VOL3_MAX                  (3400u)

#define XPOWERS_AXP2101_DCDC3_VOL_MIN                    (500)
#define XPOWERS_AXP2101_DCDC3_VOL_MAX                    (3400)

#define XPOWERS_AXP2101_DCDC3_VOL_STEPS1                 (10u)
#define XPOWERS_AXP2101_DCDC3_VOL_STEPS2                 (20u)
#define XPOWERS_AXP2101_DCDC3_VOL_STEPS3                 (100u)

#define XPOWERS_AXP2101_DCDC3_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE            (71)
#define XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE            (88)



#define XPOWERS_AXP2101_DCDC4_VOL1_MIN                  (500u)
#define XPOWERS_AXP2101_DCDC4_VOL1_MAX                  (1200u)
#define XPOWERS_AXP2101_DCDC4_VOL2_MIN                  (1220u)
#define XPOWERS_AXP2101_DCDC4_VOL2_MAX                  (1840u)

#define XPOWERS_AXP2101_DCDC4_VOL_STEPS1                 (10u)
#define XPOWERS_AXP2101_DCDC4_VOL_STEPS2                 (20u)

#define XPOWERS_AXP2101_DCDC4_VOL_STEPS1_BASE            (0u)
#define XPOWERS_AXP2101_DCDC4_VOL_STEPS2_BASE            (71)



#define XPOWERS_AXP2101_DCDC5_VOL_1200MV                 (1200)
#define XPOWERS_AXP2101_DCDC5_VOL_VAL                    (0x19)
#define XPOWERS_AXP2101_DCDC5_VOL_MIN                    (1400)
#define XPOWERS_AXP2101_DCDC5_VOL_MAX                    (3700)
#define XPOWERS_AXP2101_DCDC5_VOL_STEPS                  (100u)

#define XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN          (2600)
#define XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MAX          (3300)
#define XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS        (100)

/*
 * ----------------------- ALDO 1~4 --------------------------
 */
#define XPOWERS_AXP2101_ALDO1_VOL_MIN                    (500)
#define XPOWERS_AXP2101_ALDO1_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_ALDO1_VOL_STEPS                  (100u)

#define XPOWERS_AXP2101_ALDO2_VOL_MIN                    (500)
#define XPOWERS_AXP2101_ALDO2_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_ALDO2_VOL_STEPS                  (100u)


#define XPOWERS_AXP2101_ALDO3_VOL_MIN                    (500)
#define XPOWERS_AXP2101_ALDO3_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_ALDO3_VOL_STEPS                  (100u)


#define XPOWERS_AXP2101_ALDO4_VOL_MIN                    (500)
#define XPOWERS_AXP2101_ALDO4_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_ALDO4_VOL_STEPS                  (100u)

/*
 * ----------------------- BLDO 1~2 --------------------------
 */
#define XPOWERS_AXP2101_BLDO1_VOL_MIN                    (500)
#define XPOWERS_AXP2101_BLDO1_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_BLDO1_VOL_STEPS                  (100u)

#define XPOWERS_AXP2101_BLDO2_VOL_MIN                    (500)
#define XPOWERS_AXP2101_BLDO2_VOL_MAX                    (3500)
#define XPOWERS_AXP2101_BLDO2_VOL_STEPS                  (100u)

/*
 * ----------------------- CPUSLDO --------------------------
 */
#define XPOWERS_AXP2101_CPUSLDO_VOL_MIN                  (500)
#define XPOWERS_AXP2101_CPUSLDO_VOL_MAX                  (1400)
#define XPOWERS_AXP2101_CPUSLDO_VOL_STEPS                (50)


/*
 * ----------------------- DLDO 1~2 --------------------------
 */
#define XPOWERS_AXP2101_DLDO1_VOL_MIN                  (500)
#define XPOWERS_AXP2101_DLDO1_VOL_MAX                  (3400)
#define XPOWERS_AXP2101_DLDO1_VOL_STEPS                (100u)

#define XPOWERS_AXP2101_DLDO2_VOL_MIN                  (500)
#define XPOWERS_AXP2101_DLDO2_VOL_MAX                  (3400)
#define XPOWERS_AXP2101_DLDO2_VOL_STEPS                (100u)


#define XPOWERS_AXP2101_CONVERSION(raw)                 (22.0 + (7274 - raw) / 20.0)
#define _BV(b)											(1UL << (uint32_t)(b))

/**
 * @brief Charging led mode parameters.
 */
typedef enum __xpowers_chg_led_mode {
    XPOWERS_CHG_LED_OFF,
    XPOWERS_CHG_LED_BLINK_1HZ,
    XPOWERS_CHG_LED_BLINK_4HZ,
    XPOWERS_CHG_LED_ON,
    XPOWERS_CHG_LED_CTRL_CHG,    // The charging indicator is controlled by the charger
} xpowers_chg_led_mode_t;

/**
 * @brief axp2101 interrupt control mask parameters.
 */
typedef enum __xpowers_axp2101_irq {
    //! IRQ1 REG 40H
    XPOWERS_AXP2101_BAT_NOR_UNDER_TEMP_IRQ   = _BV(0),   // Battery Under Temperature in Work
    XPOWERS_AXP2101_BAT_NOR_OVER_TEMP_IRQ    = _BV(1),   // Battery Over Temperature in Work mode
    XPOWERS_AXP2101_BAT_CHG_UNDER_TEMP_IRQ   = _BV(2),   // Battery Under Temperature in Charge mode IRQ(bcut_irq)
    XPOWERS_AXP2101_BAT_CHG_OVER_TEMP_IRQ    = _BV(3),   // Battery Over Temperature in Charge mode IRQ(bcot_irq) enable
    XPOWERS_AXP2101_GAUGE_NEW_SOC_IRQ        = _BV(4),   // Gauge New SOC IRQ(lowsoc_irq) enable ???
    XPOWERS_AXP2101_WDT_TIMEOUT_IRQ          = _BV(5),   // Gauge Watchdog Timeout IRQ(gwdt_irq) enable
    XPOWERS_AXP2101_WARNING_LEVEL1_IRQ       = _BV(6),   // SOC drop to Warning Level1 IRQ(socwl1_irq) enable
    XPOWERS_AXP2101_WARNING_LEVEL2_IRQ       = _BV(7),   // SOC drop to Warning Level2 IRQ(socwl2_irq) enable

    //! IRQ2 REG 41H
    XPOWERS_AXP2101_PKEY_POSITIVE_IRQ        = _BV(8),   // POWERON Positive Edge IRQ(ponpe_irq_en) enable
    XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ        = _BV(9),   // POWERON Negative Edge IRQ(ponne_irq_en) enable
    XPOWERS_AXP2101_PKEY_LONG_IRQ            = _BV(10),  // POWERON Long PRESS IRQ(ponlp_irq) enable
    XPOWERS_AXP2101_PKEY_SHORT_IRQ           = _BV(11),  // POWERON Short PRESS IRQ(ponsp_irq_en) enable
    XPOWERS_AXP2101_BAT_REMOVE_IRQ           = _BV(12),  // Battery Remove IRQ(bremove_irq) enable
    XPOWERS_AXP2101_BAT_INSERT_IRQ           = _BV(13),  // Battery Insert IRQ(binsert_irq) enabl
    XPOWERS_AXP2101_VBUS_REMOVE_IRQ          = _BV(14),  // VBUS Remove IRQ(vremove_irq) enabl
    XPOWERS_AXP2101_VBUS_INSERT_IRQ          = _BV(15),  // VBUS Insert IRQ(vinsert_irq) enable

    //! IRQ3 REG 42H
    XPOWERS_AXP2101_BAT_OVER_VOL_IRQ         = _BV(16),  // Battery Over Voltage Protection IRQ(bovp_irq) enable
    XPOWERS_AXP2101_CHAGER_TIMER_IRQ         = _BV(17),  // Charger Safety Timer1/2 expire IRQ(chgte_irq) enable
    XPOWERS_AXP2101_DIE_OVER_TEMP_IRQ        = _BV(18),  // DIE Over Temperature level1 IRQ(dotl1_irq) enable
    XPOWERS_AXP2101_BAT_CHG_START_IRQ        = _BV(19),  // Charger start IRQ(chgst_irq) enable
    XPOWERS_AXP2101_BAT_CHG_DONE_IRQ         = _BV(20),  // Battery charge done IRQ(chgdn_irq) enable
    XPOWERS_AXP2101_BATFET_OVER_CURR_IRQ     = _BV(21),  // BATFET Over Current Protection IRQ(bocp_irq) enable
    XPOWERS_AXP2101_LDO_OVER_CURR_IRQ        = _BV(22),  // LDO Over Current IRQ(ldooc_irq) enable
    XPOWERS_AXP2101_WDT_EXPIRE_IRQ           = _BV(23),  // Watchdog Expire IRQ(wdexp_irq) enable

    XPOWERS_AXP2101_ALL_IRQ                  = (0xFFFFFFFFUL)
} xpowers_axp2101_irq_t;


uint32_t AXP2101_shutdown(void);
uint32_t AXP2101_disableTSPinMeasure(void);
uint32_t AXP2101_enableBattDetection(void);
uint32_t AXP2101_enableBattVoltageMeasure(void);
uint32_t AXP2101_enableSystemVoltageMeasure(void);
uint32_t AXP2101_enableVbusVoltageMeasure(void);
uint32_t AXP2101_enableIRQ(uint32_t opt);
uint32_t AXP2101_disableIRQ(uint32_t opt);
uint32_t AXP2101_clearIrqStatus(void);

uint8_t AXP2101_isDropWarningLevel1Irq(void);
uint8_t AXP2101_isVbusRemoveIrq(void);
uint8_t AXP2101_isBatInsertIrq(void);
uint8_t AXP2101_isBatRemoveIrq(void);
uint8_t AXP2101_isPekeyShortPressIrq(void);
uint8_t AXP2101_isPekeyLongPressIrq(void);
uint8_t AXP2101_isBatChargeDoneIrq(void);
uint8_t AXP2101_isBatChargeStartIrq(void);

uint32_t AXP2101_setLowBatWarnThreshold(uint8_t percentage);
uint32_t AXP2101_setLowBatShutdownThreshold(uint8_t opt);
uint32_t AXP2101_setSysPowerDownVoltage(uint16_t value);
uint32_t AXP2101_setChargingLedMode(uint8_t mode);

uint8_t AXP2101_isBatteryConnect(void);
uint8_t AXP2101_isCharging(void);
uint32_t AXP2101_getIrqStatus(uint32_t* out_value);
uint32_t AXP2101_getBatteryPercent(uint8_t* out_value);

#endif /* AXP2101_H_ */
