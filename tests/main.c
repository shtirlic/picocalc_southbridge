#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pcsb.h"
#include "jcs_regs.h"
#include "lcdspi.h"
#include "psram_spi.h"

#define SYSCLK_FREQ		200000


static const PCSB_REGS_ADDR_TABLE jcs_rat_s = JCS_PCSB_RAT;

uint32_t psram_test(psram_spi_inst_t* psram_spi){
    uint32_t psram_begin, psram_elapsed;
    float psram_speed;
    char buf[128];

    // **************** 8 bits testing ****************
    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); ++addr) {
        psram_write8(psram_spi, addr, (addr & 0xFF));
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
    lcd_printf_string("PSRAM 8MB [W-8b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); ++addr) {
        uint8_t result = psram_read8(psram_spi, addr);
        if ((uint8_t)(addr & 0xFF) != result) {
            lcd_printf_string("PSRAM>-[NOK]> failure at address %x (%x != %x)\n", addr, addr & 0xFF, result);
            return 1;
        }
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
    lcd_printf_string("PSRAM 8MB [R-8b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    // **************** 16 bits testing ****************
    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 2) {
        psram_write16(psram_spi, addr, (((addr + 1) & 0xFF) << 8) | (addr & 0xFF));
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
    lcd_printf_string("PSRAM 8MB [W-16b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 2) {
        uint16_t result = psram_read16(psram_spi, addr);
        if ((uint16_t)(
                (((addr + 1) & 0xFF) << 8) |
                (addr & 0xFF)) != result
                ) {
            lcd_printf_string("PSRAM>-[NOK]> failure at address %x (%x != %x) ", addr, (
                    (((addr + 1) & 0xFF) << 8) |
                    (addr & 0xFF)), result
            );
            return 1;
        }
    }
    psram_elapsed = (time_us_32() - psram_begin);
    psram_speed = 1000000.0 * 8 * 1024 * 1024 / psram_elapsed;
    lcd_printf_string("PSRAM 8MB [R-16b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    // **************** 32 bits testing ****************
    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 4) {
        psram_write32(
                psram_spi, addr,
                (uint32_t)(
                        (((addr + 3) & 0xFF) << 24) |
                        (((addr + 2) & 0xFF) << 16) |
                        (((addr + 1) & 0xFF) << 8)  |
                        (addr & 0XFF))
        );
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
	lcd_printf_string("PSRAM 8MB [W-32b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 4) {
        uint32_t result = psram_read32(psram_spi, addr);
        if ((uint32_t)(
                (((addr + 3) & 0xFF) << 24) |
                (((addr + 2) & 0xFF) << 16) |
                (((addr + 1) & 0xFF) << 8)  |
                (addr & 0XFF)) != result
                ) {
            lcd_printf_string("PSRAM>-[NOK]> failure at address %x (%x != %x) ", addr, (
                    (((addr + 3) & 0xFF) << 24) |
                    (((addr + 2) & 0xFF) << 16) |
                    (((addr + 1) & 0xFF) << 8)  |
                    (addr & 0XFF)), result
            );
            return 1;
        }
    }
    psram_elapsed = (time_us_32() - psram_begin);
    psram_speed = 1000000.0 * 8 * 1024 * 1024 / psram_elapsed;
	lcd_printf_string("PSRAM 8MB [R-32b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    // **************** n bits testing ****************
    uint8_t write_data[256];
    for (size_t i = 0; i < 256; ++i) {
        write_data[i] = i;
    }
    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 256) {
        for (uint32_t step = 0; step < 256; step += 16) {
            psram_write(psram_spi, addr + step, write_data + step, 16);
        }
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
	lcd_printf_string("PSRAM 8MB [W-128b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    psram_begin = time_us_32();
    uint8_t read_data[16];
    for (uint32_t addr = 0; addr < (8 * 1024 * 1024); addr += 256) {
        for (uint32_t step = 0; step < 256; step += 16) {
            psram_read(psram_spi, addr + step, read_data, 16);
            if (memcmp(read_data, write_data + step, 16) != 0) {
                sprintf(buf,"PSRAM>-[NOK]> failure at address %x", addr);
                printf("%s", buf);
                lcd_print_string(buf);
                return 1;
            }
        }
    }
    psram_elapsed = time_us_32() - psram_begin;
    psram_speed = 1000000.0 * 8 * 1024.0 * 1024 / psram_elapsed;
	lcd_printf_string("PSRAM 8MB [R-128b] - %d ms - %d KB/s\n", psram_elapsed / 1000, (uint32_t)psram_speed / 1000);

    lcd_print_string("PSRAM>-[OK]-\n");
	return 0;
}

static void RTC_disp(PCSB_RTC_DATE_STRUCT* const rtc_date, PCSB_RTC_TIME_STRUCT* const rtc_time) {
    int result = 0;

    result = pcsb_rtc_read_date(rtc_date);
    if (result == 0)
        lcd_printf_string("RTC check: %s %02d/%02d/%04d", &dayOfWeekLUT[rtc_date->dayOfWeek-1], rtc_date->day, rtc_date->month, 2000 + rtc_date->year);
    else {
        lcd_print_string("SB-RTC>-[NOK]> R-REG-DATE!\n");
        lcd_printf_string("RAW: %08X Err: %d\n", rtc_date->raw, result);
    }

    result = pcsb_rtc_read_time(rtc_time);
    if (result == 0)
        lcd_printf_string(" - %02d:%02d:%02d\n", rtc_time->hours, rtc_time->minutes, rtc_time->secondes);
    else {
        lcd_print_string("\nSB-RTC>-[NOK]> R-REG-TIME!\n");
        lcd_printf_string("RAW: %08X Err: %d\n", rtc_time->raw, result);
    }
}

int main() {
	uint32_t ret_state = 0;
    uint8_t i2c_err_cnt = 0;
    PCSB_RTC_DATE_STRUCT rtc_date = {0};
    PCSB_RTC_TIME_STRUCT rtc_time = {0};
	
	// Configure primary and auxilliaries clocks
#if SYSCLK_FREQ > 175000
    vreg_set_voltage(VREG_VOLTAGE_1_15);
#endif
    set_sys_clock_khz(SYSCLK_FREQ, true);
	clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        SYSCLK_FREQ * 1000,                               // Input frequency
        SYSCLK_FREQ * 1000                                // Output (must be same as no divider)
    );
	
	// Init the drivers after clocks are locked
    stdio_init_all();
    sleep_ms(500);

    uint32_t pcsb_state = pcsb_init(&jcs_rat_s, JCS_PCSB_ID);
    int boot_c = pcsb_kbd_read_c();
	
	ret_state = lcd_init();
	lcd_clear();
	lcd_print_string("If you can read this...\nYou should make more coffee!\n\n");
    lcd_printf_string("> SB-I2C speed: %d KHz\n", pcsb_get_i2c_speed() / 1000);
    lcd_printf_string("> LCD-SPI speed: %d KHz\n", ret_state / 1000);
	
	if (pcsb_state == 2)
		lcd_print_string("SB-CORE>-[NOK]> MM-REG-ID!\n");   // MissMatch
	else if (pcsb_state == 1)
		lcd_print_string("SB-CORE>-[NOK]> R-REG-ID!\n");
	else
		lcd_print_string("SB-CORE>-[OK]-\n");

    if (boot_c > 0)
        lcd_printf_string("SB-KBD> Boot press: 0x%02X\n", boot_c);

	//const I2C_KBD_REGS_ADDR_TABLE* const curr_kbd_table = pcsb_get_addr_table();
	//lcd_printf_string("  KBD FIFO addr: 0x%02X\n", curr_kbd_table->reg_fif);    

	RTC_disp(&rtc_date, &rtc_time);
	if ((rtc_date.raw & 0xFFFFFF00) == 0x00010100) {
		rtc_date.day = 31;
		rtc_date.month = 05;
		rtc_date.year = 25;
		if (pcsb_rtc_set_date(&rtc_date) == -2)
			lcd_print_string("SB-RTC>-[NOK]> W-REG-DATE\n");
		else {
            rtc_time.hours = 23;
            rtc_time.minutes = 59;
            if (pcsb_rtc_set_time(&rtc_time) == -2)
                lcd_print_string("SB-RTC>-[NOK]> W-REG-TIME\n");
            else {
                lcd_print_string("SB-RTC>-[OK]> RTC reconfigured\n");

                RTC_disp(&rtc_date, &rtc_time);
            }
        }
	} else
		lcd_print_string("SB-RTC>-[OK]-\n");

    // External PSRAM testing
	//psram_spi_inst_t psram_spi = psram_spi_init_clkdiv(pio1, -1, 1.0f,true);
    //psram_test(&psram_spi);

    // Test loop
    lcd_print_string("SB-KBD> Test mode. Press Esc to pass.\n");
    for ( ;; ) {
		int c = pcsb_kbd_read_c();

        if (c == 0xB1)  // ESC
            break;

        if (c == -12 || c == -11) {
            if (i2c_err_cnt <= 7) {
			    lcd_printf_string("KBD FIFO read failure! (%d)\n", c);
                ++i2c_err_cnt;
            }
            sleep_ms(2000);
		} else if(c != -1 && c > 0) {
            if (i2c_err_cnt > 7) {
                lcd_printf_string("SB-KBD> link restored.\n", c);
                i2c_err_cnt = 0;
            }
			lcd_printf_string("SB-KBD> 0x%02X\n", c);
		}
		sleep_ms(20);
    }

    // Stop w/ 5s delay
    if (pcsb_pwr_sleep(5) == 0) {
        lcd_print_string("SB-CORE> Sleep requested in 5s...\n");
        sleep_ms(800);
        lcd_print_string("SB-CORE> 4\n");
        sleep_ms(1000);
        lcd_print_string("SB-CORE> 3\n");
        sleep_ms(1000);
        lcd_print_string("SB-CORE> 2\n");
        sleep_ms(1000);
        lcd_print_string("SB-CORE> 1\n");
        sleep_ms(1000);
        lcd_print_string("SB-CORE> 0\n");
        sleep_ms(2000);
        lcd_print_string("If you read this, it's a failure.\n");
    } else
        lcd_print_string("SB-CORE> Sleep request failure!\n");
}
