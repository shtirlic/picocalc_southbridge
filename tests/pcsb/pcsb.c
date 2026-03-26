#include "pcsb_int.h"
#include <hardware/i2c.h>

#define PCSB_I2C_SDA    6
#define PCSB_I2C_SCL    7

#define PCSB_I2C_SPEED  100000


static uint8_t pcsb_inited = 0;
static uint i2c_speed = 0;


static const PCSB_REGS_ADDR_TABLE addr_table_s = PROG_PCSB_RAT;
static const PCSB_REGS_ADDR_TABLE* addr_table = &addr_table_s;

uint32_t pcsb_init(const PCSB_REGS_ADDR_TABLE* const addr_table_override, const uint8_t req_typ_id) {
	uint32_t result = 0;
	
    i2c_deinit(PCSB_I2C_MOD);
    gpio_set_function(PCSB_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PCSB_I2C_SDA, GPIO_FUNC_I2C);
    i2c_speed = i2c_init(PCSB_I2C_MOD, PCSB_I2C_SPEED);

    pcsb_inited = 1;
	
	uint8_t typ_id = 0;
    if (addr_table_override != NULL) {
        if (pcsb_read_reg8b(0x00, &typ_id) == 0) {
            if (req_typ_id == typ_id)
                addr_table = addr_table_override;
            else
                result = 2;
        } else {
            result = 1;
        }
    }
	
	return result;
}

inline uint8_t pcsb_is_init(void) {
    return (pcsb_inited == 1);
}

inline uint pcsb_get_i2c_speed(void) {
    return i2c_speed;
}

uint8_t pcsb_read_version(void) {
    uint8_t v = 0;
    pcsb_read_reg8b(PCSB_REG_VER, &v);
    return v;
}

const PCSB_REGS_ADDR_TABLE* pcsb_get_addr_table(void) {
    return addr_table;
}

int _int_i2c_op(const uint8_t* in_buff, const size_t in_size, uint8_t* out_buff, const size_t out_size) {
    int retval = 0;

    if (in_buff == NULL || out_buff == NULL || in_size <= 0 || out_size <= 0)
        return 1;

    retval = i2c_write_timeout_us(PCSB_I2C_MOD, PCSB_I2C_ADDR, in_buff, in_size, false, 200000);
    if (retval == PICO_ERROR_GENERIC || retval == PICO_ERROR_TIMEOUT)
        return retval;

    //sleep_us(100);
    //sleep_ms(16);
    retval = i2c_read_timeout_us(PCSB_I2C_MOD, PCSB_I2C_ADDR, out_buff, out_size, false, 200000);
    if (retval == PICO_ERROR_GENERIC || retval == PICO_ERROR_TIMEOUT)
        return retval;

    sleep_us(200);
    //sleep_ms(16);
    return 0;
}

uint32_t pcsb_read_reg8b(const uint8_t reg_addr, uint8_t* const out_buff) {
	uint8_t msg[2];
	msg[0] = reg_addr;
	
    if (_int_i2c_op(msg, 1, msg, 2) != 0)
        return 1;

    *out_buff = msg[1];
	return 0;
}

int pcsb_read_battery(void) {
    uint16_t buff = 0;
    unsigned char msg[2];

    if (pcsb_inited == 0) return -1;
	msg[0] = addr_table->reg_bat;

    if (_int_i2c_op(msg, 1, (uint8_t*)&buff, 2) != 0)
        return -1;

    if (buff != 0)
        return buff;

    return -1;
}

int pcsb_set_backlight(PCSB_BKL target, uint8_t val) {
    uint16_t buff = 0;
    unsigned char msg[2];

    if (pcsb_is_init() == 0) return -1;
	msg[0] = target == PCSB_BKL_KBD ? pcsb_get_addr_table()->reg_bk2 : pcsb_get_addr_table()->reg_bkl;
	msg[1] = val;
    bitSet(msg[0],7);

    if (_int_i2c_op(msg, 2, (uint8_t*)&buff, 2) != 0)
        return -1;

    if (buff != 0)
        return buff;

    return -1;
}

int pcsb_pwr_off(uint8_t delay) {
    unsigned char ans[2] = {0};
    unsigned char msg[2];

    if (pcsb_is_init() == 0) return -1;
    msg[0] = pcsb_get_addr_table()->reg_off_ctrl;
	msg[1] = (1 << 7) | (delay & 0x3F);
    bitSet(msg[0],7);

    if (_int_i2c_op(msg, 2, ans, 2) != 0)
        return -1;

    if (ans[1] != msg[1])
        return -2;

    return 0;
}

int pcsb_pwr_sleep(uint8_t delay) {
    unsigned char ans[2] = {0};
    unsigned char msg[2];

    if (pcsb_is_init() == 0) return -1;
    msg[0] = pcsb_get_addr_table()->reg_off_ctrl;
	msg[1] = (1 << 6) | (delay & 0x3F);
    bitSet(msg[0],7);

    if (_int_i2c_op(msg, 2, ans, 2) != 0)
        return -1;

    if (ans[1] != msg[1])
        return -2;

    return 0;
}
