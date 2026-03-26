#include "kbd.h"
#include <hardware/i2c.h>
#include "pcsb_int.h"


int pcsb_kbd_read_c() {
    static int ctrlheld = 0;
    uint16_t buff = 0;
    unsigned char msg[2];
    int c = -1;

    if (pcsb_is_init() == 0) return -1;
	msg[0] = pcsb_get_addr_table()->reg_fif;

    c = _int_i2c_op(msg, 1, (uint8_t*)&buff, 2);
    if (c != 0)
        return c - 10;

    if (buff != 0) {
        if (buff == 0x7e03)
            ctrlheld = 0;
        else if (buff == 0x7e02) {
            ctrlheld = 1;
        } else if ((buff & 0xff) == 1) {//pressed
            c = buff >> 8;
            int realc = -1;
            switch (c) {
                default:
                    realc = c;
                    break;
            }
            c = realc;
            if (c >= 'a' && c <= 'z' && ctrlheld)
                c = c - 'a' + 1;
        }
        return c;
    }
    return -1;
}
