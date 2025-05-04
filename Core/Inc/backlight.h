#include "stm32f1xx_hal.h"


#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_

void lcd_backlight_update_from_reg(void);
void lcd_backlight_on(void);
void lcd_backlight_off(void);
void lcd_backlight_update(uint8_t step);
void lcd_backlight_update_up(void);
void lcd_backlight_update_down(void);

void kbd_backlight_on(void);
void kbd_backlight_off(void);
void kbd_backlight_update(uint8_t step);
void kbd_backlight_update_loop(void);

#endif /* BACKLIGHT_H_ */
