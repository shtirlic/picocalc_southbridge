#include "stm32f1xx_hal.h"


#ifndef RTC_H_
#define RTC_H_

void i2cs_fill_buffer_RTC_date(uint8_t* const buff, const volatile RTC_DateTypeDef* const date_s);
void i2cs_fill_buffer_RTC_time(uint8_t* const buff, const volatile RTC_TimeTypeDef* const time_s);
void i2cs_RTC_date_from_buffer(volatile RTC_DateTypeDef* const date_s, const uint8_t* const buff);
void i2cs_RTC_time_from_buffer(volatile RTC_TimeTypeDef* const time_s, const uint8_t* const buff);

uint32_t rtc_run_alarm(void);
uint32_t rtc_stop_alarm(void);

#endif /* RTC_H_ */
