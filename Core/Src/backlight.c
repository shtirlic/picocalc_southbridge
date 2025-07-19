#include "backlight.h"

#include "hal_interface.h"
#include "regs.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// LCD backlight curve based on brightness measurements for specific value of PWM duty cycle.
// Using this, I've established a custom command curve.

//static const uint16_t lcd_bckl_steps[LCD_BCKL_STEPS] = {20, 60, 96, 134, 166, 192, 210, 256, 358, 460};
static const uint16_t lcd_bckl_steps[LCD_BCKL_STEPS] = {10, 20, 40, 60, 80, 110, 150, 200, 256, 440};
static const uint16_t kbd_bckl_steps[KBD_BCKL_STEPS] = {0, 40, 110, 260};


inline void lcd_backlight_update_from_reg(void) {
	uint16_t val = 0;

	val = lcd_bckl_steps[reg_get_value(REG_ID_BKL) % LCD_BCKL_STEPS];
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);
}

inline void lcd_backlight_on(void) {
	lcd_backlight_update_from_reg();
}

inline void lcd_backlight_off(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0x0000);
}

inline void lcd_backlight_update(uint8_t step) {
	uint16_t val = 0;
	const uint8_t index = step % LCD_BCKL_STEPS;

	// In case the user ask for a 0-255 range (like in the official driver), convert to the 9-bit (0-511) range
	if (step >= LCD_BCKL_STEPS)
		val = (uint16_t)((step - lcd_bckl_steps[0]) * 511 / 255) % 512;
	else
		val = lcd_bckl_steps[index];

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);

	reg_set_value(REG_ID_BKL, index);
}

inline void lcd_backlight_update_up(void) {
	uint16_t val = 0;
	uint8_t index = reg_get_value(REG_ID_BKL) % LCD_BCKL_STEPS;

	index = index < (LCD_BCKL_STEPS-1) ? index + 1 : (LCD_BCKL_STEPS-1);
	val = lcd_bckl_steps[index];
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);

	reg_set_value(REG_ID_BKL, index);
}

inline void lcd_backlight_update_down(void) {
	uint16_t val = 0;
	uint8_t index = reg_get_value(REG_ID_BKL) % LCD_BCKL_STEPS;

	index = index > 0 ? index - 1 : 0;
	val = lcd_bckl_steps[index];
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);

	reg_set_value(REG_ID_BKL, index);
}


inline void kbd_backlight_on(void) {
	uint16_t val = 0;

	val = kbd_bckl_steps[reg_get_value(REG_ID_BK2) % KBD_BCKL_STEPS];
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, val);
}

inline void kbd_backlight_off(void) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0x0000);
}

inline void kbd_backlight_update(uint8_t step) {
	uint16_t val = 0;
	const uint8_t index = step % KBD_BCKL_STEPS;

	// In case the user ask for a 0-255 range (like in the official driver), convert to the 9-bit (0-511) range
	if (step >= KBD_BCKL_STEPS)
			val = (uint16_t)((step - kbd_bckl_steps[0]) * 511 / 255) % 512;
		else
			val = kbd_bckl_steps[index];

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, val);

	reg_set_value(REG_ID_BK2, index);
}

inline void kbd_backlight_update_loop(void) {
	uint16_t val = 0;
	uint8_t index = reg_get_value(REG_ID_BK2) % KBD_BCKL_STEPS;

	index = index < (KBD_BCKL_STEPS-1) ? index + 1 : 0;
	val = kbd_bckl_steps[index];
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, val);

	reg_set_value(REG_ID_BK2, index);
}
