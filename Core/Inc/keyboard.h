#include "stm32f1xx_hal.h"

#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

enum key_state {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED,
    KEY_STATE_HOLD,
    KEY_STATE_RELEASED,
};

#define NUM_OF_COLS		8
#define NUM_OF_ROWS		7

#define KEY_LIST_SIZE	10

#define KEY_POLL_TIME	16		// in ms, scanning freq
#define KEY_HOLD_TIME	300		// in ms, delay before switching to "rapid-trigger" mode
#define KEY_REPEAT_TIME	100		// in ms, delay between each triggers in "rapid-trigger" mode

#define KEY_JOY_UP      0x01
#define KEY_JOY_DOWN    0x02
#define KEY_JOY_LEFT    0x03
#define KEY_JOY_RIGHT   0x04
#define KEY_JOY_CENTER  0x05
#define KEY_BTN_LEFT1   0x06
#define KEY_BTN_RIGHT1  0x07

#define KEY_BACKSPACE   0x08
#define KEY_TAB         0x09
#define KEY_ENTER       0x0A
// 0x0D - CARRIAGE RETURN
#define KEY_BTN_LEFT2   0x11
#define KEY_BTN_RIGHT2  0x12


#define KEY_MOD_ALT     0xA1
#define KEY_MOD_SHL     0xA2
#define KEY_MOD_SHR     0xA3
#define KEY_MOD_SYM     0xA4
#define KEY_MOD_CTRL    0xA5

#define KEY_ESC         0xB1
#define KEY_UP          0xb5
#define KEY_DOWN        0xb6
#define KEY_LEFT        0xb4
#define KEY_RIGHT       0xb7

#define KEY_BREAK       0xd0 // == KEY_PAUSE
#define KEY_INSERT      0xD1
#define KEY_HOME        0xD2
#define KEY_DEL         0xD4
#define KEY_END         0xD5
#define KEY_PAGE_UP     0xd6
#define KEY_PAGE_DOWN   0xd7

#define KEY_CAPS_LOCK   0xC1

#define KEY_F1 0x81
#define KEY_F2 0x82
#define KEY_F3 0x83
#define KEY_F4 0x84
#define KEY_F5 0x85
#define KEY_F6 0x86
#define KEY_F7 0x87
#define KEY_F8 0x88
#define KEY_F9 0x89
#define KEY_F10 0x90


extern uint8_t io_matrix[9];
extern uint8_t js_bits;

typedef void (*key_callback)(char, enum key_state);
typedef void (*lock_callback)(uint8_t, uint8_t);

void keyboard_set_key_callback(key_callback callback);
void keyboard_set_lock_callback(lock_callback callback);
uint8_t keyboard_get_capslock(void);
uint8_t keyboard_get_numlock(void);
uint16_t keyboard_get_hold_period(void);
void keyboard_set_hold_period(uint16_t);
uint8_t keyboard_get_shift(void);
uint8_t keyboard_get_alt(void);
void keyboard_process(void);


#ifdef __cplusplus
}
#endif

#endif /* KEYBOARD_H_ */
