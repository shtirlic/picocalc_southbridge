#include "keyboard.h"

#include "hal_interface.h"
#include "regs.h"
#include "backlight.h"
#include "batt.h"


enum mod {
	MOD_NONE = 0,
	MOD_SYM,
	MOD_ALT,
	MOD_SHL,
	MOD_SHR,
	MOD_CTRL,
	MOD_LAST,
};

struct entry {
	uint8_t chr;
	uint8_t symb;
	enum mod mod;
};
#define INIT_ENTRY1(a)		{a,a,MOD_NONE}
#define INIT_ENTRY2(a,b)	{a,b,MOD_NONE}

struct list_item {
    const struct entry *p_entry;
    uint32_t hold_start_time;
    uint32_t last_repeat_time;
    enum key_state state;
    uint8_t mods[MOD_LAST];
};

struct gpio_pin {
	GPIO_TypeDef *GPIOx;
	uint32_t PinMask;
};



static const struct entry kbd_entries[][NUM_OF_COLS] = {
  {INIT_ENTRY2(KEY_F5,KEY_F10),			INIT_ENTRY2(KEY_F4,KEY_F9),  INIT_ENTRY2(KEY_F3,KEY_F8),INIT_ENTRY2(KEY_F2,KEY_F7),   INIT_ENTRY2(KEY_F1,KEY_F6),    INIT_ENTRY2('`','~'),INIT_ENTRY2('3','#'), INIT_ENTRY2('2','@')},
  {INIT_ENTRY1(KEY_BACKSPACE),			INIT_ENTRY2(KEY_DEL,KEY_END),INIT_ENTRY1(KEY_CAPS_LOCK),INIT_ENTRY2(KEY_TAB,KEY_HOME),INIT_ENTRY2(KEY_ESC,KEY_BREAK),INIT_ENTRY2('4','$'),INIT_ENTRY1('E'),     INIT_ENTRY1('W')},
  {INIT_ENTRY1('P'),					INIT_ENTRY2('=','+'),        INIT_ENTRY2('-','_'),      INIT_ENTRY2('\\','|'),        INIT_ENTRY2('/','?'),          INIT_ENTRY1('R'),    INIT_ENTRY1('S'),     INIT_ENTRY2('1','!')},
  {INIT_ENTRY2(KEY_ENTER,KEY_INSERT),	INIT_ENTRY2('8','*'),        INIT_ENTRY2('7','&'),      INIT_ENTRY2('6','^'),         INIT_ENTRY2('5','%'),          INIT_ENTRY1('F'),    INIT_ENTRY1('X'),     INIT_ENTRY1('Q')},
  {INIT_ENTRY2('.','>'),				INIT_ENTRY1('I'),            INIT_ENTRY1('U'),          INIT_ENTRY1('Y'),             INIT_ENTRY1('T'),              INIT_ENTRY1('V'),    INIT_ENTRY2(';',':'), INIT_ENTRY1('A')},
  {INIT_ENTRY1('L'),					INIT_ENTRY1('K'),            INIT_ENTRY1('J'),          INIT_ENTRY1('H'),             INIT_ENTRY1('G'),              INIT_ENTRY1('C'),    INIT_ENTRY2('\'','"'),INIT_ENTRY1('Z')},
  {INIT_ENTRY1('O'),					INIT_ENTRY2(',','<'),        INIT_ENTRY1('M'),          INIT_ENTRY1('N'),             INIT_ENTRY1('B'),              INIT_ENTRY1('D'),    INIT_ENTRY1(' '),     INIT_ENTRY1(0x00)}
};

static const struct entry btn_entries[12] = {
	{.mod = MOD_ALT},
	{.mod = MOD_CTRL},
	{.mod = MOD_SHL},
	{.mod = MOD_SHR},
	INIT_ENTRY2('0',')'),
	INIT_ENTRY2('9','('),
	INIT_ENTRY2(']','}'),
	INIT_ENTRY2('[','{'),
	INIT_ENTRY1(KEY_RIGHT),
	INIT_ENTRY2(KEY_UP,KEY_PAGE_UP),
	INIT_ENTRY2(KEY_DOWN,KEY_PAGE_DOWN),
	INIT_ENTRY1(KEY_LEFT)
};

static const struct gpio_pin row_pins[NUM_OF_ROWS] = {
	{ROW_1_GPIO_Port, ROW_1_Pin},
	{ROW_2_GPIO_Port, ROW_2_Pin},
	{ROW_3_GPIO_Port, ROW_3_Pin},
	{ROW_4_GPIO_Port, ROW_4_Pin},
	{ROW_5_GPIO_Port, ROW_5_Pin},
	{ROW_6_GPIO_Port, ROW_6_Pin},
	{ROW_7_GPIO_Port, ROW_7_Pin}
};

static const struct gpio_pin col_pins[NUM_OF_COLS] = {
	{COL_1_GPIO_Port, COL_1_Pin},
	{COL_2_GPIO_Port, COL_2_Pin},
	{COL_3_GPIO_Port, COL_3_Pin},
	{COL_4_GPIO_Port, COL_4_Pin},
	{COL_5_GPIO_Port, COL_5_Pin},
	{COL_6_GPIO_Port, COL_6_Pin},
	{COL_7_GPIO_Port, COL_7_Pin},
	{COL_8_GPIO_Port, COL_8_Pin}
};

static const struct gpio_pin btn_pins[12] = {
	{KEY_1_GPIO_Port, KEY_1_Pin},
	{KEY_2_GPIO_Port, KEY_2_Pin},
	{KEY_3_GPIO_Port, KEY_3_Pin},
	{KEY_4_GPIO_Port, KEY_4_Pin},
	{KEY_5_GPIO_Port, KEY_5_Pin},
	{KEY_6_GPIO_Port, KEY_6_Pin},
	{KEY_7_GPIO_Port, KEY_7_Pin},
	{KEY_8_GPIO_Port, KEY_8_Pin},
	{KEY_9_GPIO_Port, KEY_9_Pin},
	{KEY_10_GPIO_Port, KEY_10_Pin},
	{KEY_11_GPIO_Port, KEY_11_Pin},
	{KEY_12_GPIO_Port, KEY_12_Pin}
};

static struct list_item keys_list[KEY_LIST_SIZE];
static lock_callback _lock_callback = NULL;
static key_callback _key_callback= NULL;
static uint32_t last_process_time = 0;
static uint8_t mods[MOD_LAST];
static uint8_t capslock_changed = 0;
static uint8_t capslock = 0;
static uint8_t numlock_changed = 0;
static uint8_t numlock = 0;
static uint16_t hold_period = KEY_HOLD_TIME;

uint8_t io_matrix[9] = {0};		//for IO matrix,last byte is the restore key(c64 only)
uint8_t js_bits = 0xFF;			// c64 joystick bits


inline void keyboard_set_key_callback(key_callback callback) {
	_key_callback = callback;
}

inline void keyboard_set_lock_callback(lock_callback callback) {
	_lock_callback = callback;
}

inline uint8_t keyboard_get_capslock(void) {
	return capslock & 0x1;
}

inline uint8_t keyboard_get_numlock(void) {
	return numlock & 0x1;
}

inline uint16_t keyboard_get_hold_period(void) {
	return hold_period;
}

inline void keyboard_set_hold_period(uint16_t value) {
	hold_period = value;
}

static void transition_to(struct list_item * const p_item, const enum key_state next_state) {
	uint8_t output = 1;
	const struct entry * const p_entry = p_item->p_entry;

	p_item->state = next_state;

	if (!_key_callback || !p_entry)
		return;

	char chr = p_entry->chr;

	switch (p_entry->mod) {
	case MOD_ALT:
		if (reg_is_bit_set(REG_ID_CFG, CFG_REPORT_MODS))
			chr = KEY_MOD_ALT;
		break;

    case MOD_SHL:
		if (reg_is_bit_set(REG_ID_CFG, CFG_REPORT_MODS))
			chr = KEY_MOD_SHL;
		break;

    case MOD_SHR:
		if (reg_is_bit_set(REG_ID_CFG, CFG_REPORT_MODS))
			chr = KEY_MOD_SHR;
		break;

    case MOD_SYM:
		if (reg_is_bit_set(REG_ID_CFG, CFG_REPORT_MODS))
			chr = KEY_MOD_SYM;
		break;

    case MOD_CTRL:
		if (reg_is_bit_set(REG_ID_CFG, CFG_REPORT_MODS))
			chr = KEY_MOD_CTRL;
		break;

    default:	//toggle operation
    	if (chr == KEY_CAPS_LOCK && next_state == KEY_STATE_PRESSED) {
    		capslock = (capslock & 0x1) ^ 0x1;
    		capslock_changed = 1;
		}

    	if (reg_is_bit_set(REG_ID_CFG, CFG_USE_MODS)) {
			const uint8_t shift = (mods[MOD_SHL] || mods[MOD_SHR]);
			const uint8_t alt = mods[MOD_ALT] | numlock;
			//const uint8_t ctrl = mods[MOD_CTRL];

			if (shift && (chr <'A' || chr >'Z')) {
				chr = p_entry->symb;
			} else if (capslock && (chr >= 'A' && chr <= 'Z')) {
				//pass
			} else if (alt) {
				//ctrl for operators
				if (next_state == KEY_STATE_PRESSED) {
					if (chr == ',' || chr == '.' || chr == ' ' || chr == 'B') {
						output = 0;
					} else if (chr == 'I') {
						output = 1;
						chr = KEY_INSERT;
					}
				}

				if (next_state == KEY_STATE_RELEASED) {
					if (chr == ',' || chr == '.' || chr == ' ' || chr == 'B') {
						output = 0;
					} else if(chr == 'I'){
						output = 1;
						chr = KEY_INSERT;
					}
				}

				if(next_state == KEY_STATE_RELEASED) {
					if(chr ==',')
						lcd_backlight_update_down();
					else if(chr =='.')
						lcd_backlight_update_up();
					else if(chr == ' ')
						//loop update keyboard backlight
						kbd_backlight_update_loop();
					else if(chr == 'B')
						show_bat_segs();
				}
			} else if (!shift && (chr >= 'A' && chr <= 'Z')) {
				chr = (chr + ' ');	// uppercase to lowercase for a to z
			}
    	}
    	break;
	}

	if (chr != 0 && output == 1) {
		if(next_state == KEY_STATE_HOLD &&
				((chr >= 32 && chr <= 127) || chr == KEY_ENTER || chr == KEY_TAB || chr == KEY_DEL || chr == KEY_BACKSPACE  || chr == KEY_UP || chr == KEY_DOWN || chr == KEY_RIGHT || chr == KEY_LEFT) )
			_key_callback(chr, KEY_STATE_PRESSED);
		else
			_key_callback(chr, next_state);
	}
}

static void next_item_state(struct list_item* const p_item, const uint8_t pressed) {
	switch (p_item->state) {
	default:
	case KEY_STATE_IDLE:
		if (pressed) {
			if (p_item->p_entry->mod != MOD_NONE)
				mods[p_item->p_entry->mod] = 1;

			if (!capslock_changed && mods[MOD_SHR] && mods[MOD_ALT]) {
				capslock = 1;
				capslock_changed = 1;
			}

			if (!numlock_changed && mods[MOD_SHL] && mods[MOD_ALT]) {
				numlock = 1;
				numlock_changed = 1;
			}

			if (!capslock_changed && (mods[MOD_SHL] || mods[MOD_SHR])) {
				capslock = 0;
				capslock_changed = 1;
			}

			if (!numlock_changed && (mods[MOD_SHL] || mods[MOD_SHR])) {
				numlock = 0;
				numlock_changed = 1;
			}

			if (!mods[MOD_ALT]) {
				capslock_changed = 0;
				numlock_changed = 0;
			}

			if (_lock_callback && (capslock_changed || numlock_changed))
				_lock_callback(capslock_changed, numlock_changed);

			transition_to(p_item, KEY_STATE_PRESSED);

			p_item->hold_start_time = uptime_ms();
		}
		break;

	case KEY_STATE_PRESSED:
		if (uptime_ms() - p_item->hold_start_time > hold_period) {
			transition_to(p_item, KEY_STATE_HOLD);
			p_item->last_repeat_time = uptime_ms();
		} else if (!pressed)
			transition_to(p_item, KEY_STATE_RELEASED);
		break;

	case KEY_STATE_HOLD:
		if (!pressed)
			transition_to(p_item, KEY_STATE_RELEASED);
		else {
			if (uptime_ms() - p_item->hold_start_time > hold_period) {
				if(uptime_ms() - p_item->last_repeat_time > KEY_REPEAT_TIME) {
					transition_to(p_item, KEY_STATE_HOLD);
					p_item->last_repeat_time = uptime_ms();
				}
			}
		}
		break;

	case KEY_STATE_RELEASED:
		if (p_item->p_entry->mod != MOD_NONE)
			mods[p_item->p_entry->mod] = 0;

		p_item->p_entry = NULL;
		transition_to(p_item, KEY_STATE_IDLE);
		break;
	}
}

void keyboard_process(void) {
	js_bits = 0xFF;

	if (uptime_ms() - last_process_time <= reg_get_value(REG_ID_FRQ))
		return;

	// Scan for columns
	for (uint8_t c = 0; c < NUM_OF_COLS; ++c) {
		uint8_t col_value = 0;
		// Enable the columns signal - OD logic
		LL_GPIO_ResetOutputPin(col_pins[c].GPIOx, col_pins[c].PinMask);

		// Scan for rows
		for (uint8_t r = 0; r < NUM_OF_ROWS; ++r) {
			const uint8_t pressed = (LL_GPIO_IsInputPinSet(row_pins[r].GPIOx, row_pins[r].PinMask) == 0);
			uint8_t row_bit = (1 << r);

			if (pressed) {
				if (c == 1 && r == 4)
					js_bits &= ~row_bit;
				col_value &= ~row_bit;
			} else {
				if (c == 1 && r == 4) {
					js_bits |= row_bit;
				}
				col_value |= row_bit;
			}

			const int32_t key_idx = (int32_t)((r * NUM_OF_COLS) + c);
			int32_t list_idx = -1;
			for (int32_t i = 0; i < KEY_LIST_SIZE; ++i) {
				if (keys_list[i].p_entry != &((const struct entry*)kbd_entries)[key_idx])
					continue;

				list_idx = i;
				break;
			}

			if (list_idx > -1) {
				next_item_state(&keys_list[list_idx], pressed);
				continue;
			}

			if (!pressed)
				continue;

			for (uint32_t i = 0; i < KEY_LIST_SIZE; ++i) {
				if (keys_list[i].p_entry != NULL)
				  continue;

				keys_list[i].p_entry = &((const struct entry*)kbd_entries)[key_idx];
				keys_list[i].state = KEY_STATE_IDLE;
				next_item_state(&keys_list[i], pressed);

				break;
			}
		}
		// Disable the columns signal - OD logic
		LL_GPIO_SetOutputPin(col_pins[c].GPIOx, col_pins[c].PinMask);

		io_matrix[c] = col_value;
		for (uint8_t b = 0; b < 12; ++b) {
			const uint8_t pressed = (LL_GPIO_IsInputPinSet(btn_pins[b].GPIOx, btn_pins[b].PinMask) == 0);
			if (b < 8) {	// read BTN1->BTN8
				if (pressed)
					io_matrix[b] &= (uint8_t)(~(1 << 7));
				else
					io_matrix[b] |= (1 << 7);
			} else {		//c64 joystick arrow keys
				//B12=left,, B11=down,B10 = up,B9 = right
				if (pressed)
					js_bits &= (uint8_t)(~(1 << (b - 8)));
				else
					js_bits |= (1 << (b - 8));
			}

			int8_t list_idx = -1;
			for (int8_t i = 0; i < KEY_LIST_SIZE; ++i) {
				if (keys_list[i].p_entry != &((const struct entry*)btn_entries)[b])
					continue;

				list_idx = i;
					break;
			}

			if (list_idx > -1) {
				next_item_state(&keys_list[list_idx], pressed);
				continue;
			}

			if (!pressed)
				continue;

		    for (uint8_t i = 0 ; i < KEY_LIST_SIZE; ++i) {
				if (keys_list[i].p_entry != NULL)
					continue;

				keys_list[i].p_entry = &((const struct entry*)btn_entries)[b];
				keys_list[i].state = KEY_STATE_IDLE;
				next_item_state(&keys_list[i], pressed);

				break;
		    }
		}

		io_matrix[8] = 0xFF;
	}

	last_process_time = uptime_ms();
}
