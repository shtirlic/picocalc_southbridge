#include "rtc.h"

#include "hal_interface.h"
#include "regs.h"


extern RTC_HandleTypeDef hrtc;

static uint32_t date_sync_counter = 0;

extern void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	uint8_t rtc_conf = reg_get_value(REG_ID_RTC_CFG);
	RTC_DateTypeDef date_s = {0};
	uint8_t date_valid = 1;

	// Needed to fetch the good date from wake-up
	force_rtc_bck_load();

	if ((rtc_conf & RTC_CFG_DATE_ALARM) == RTC_CFG_DATE_ALARM) {
		HAL_RTC_GetDate(hrtc, &date_s, RTC_FORMAT_BIN);
		if (date_s.Year != rtc_alarm_date._s.Year ||
			date_s.Month != rtc_alarm_date._s.Month ||
			date_s.Date != rtc_alarm_date._s.Date)
				date_valid = 0;
	}

	rtc_stop_alarm();
	if (date_valid == 1) {
		if ((rtc_conf & RTC_CFG_REARM) == RTC_CFG_REARM)
			rtc_run_alarm();
	} else {
		rtc_run_alarm();
	}
}

static uint32_t date_is_anterior(RTC_DateTypeDef* d1, RTC_DateTypeDef* d2) {
	if (d1->Year < d2->Year)
		return 1;
	if (d1->Year > d2->Year)
		return 0;

	if (d1->Month < d2->Month)
		return 1;
	if (d1->Month > d2->Month)
		return 0;

	if (d1->Date < d2->Date)
		return 1;

	return 0;
}

// STM32F1 RTC is really basic and don't remember calendar!
// At wake-up from any low-power mode, we need to call this method before
// any HAL_RTC_GetDate or HAL_RTC_GetTime functions. This is mandatory
// to compute the right date.
void force_rtc_bck_load(void) {
	RTC_DateTypeDef_u bckDate;
	bckDate.raw = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) & 0xFFFF) | (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) << 16);

	if (IS_RTC_YEAR(bckDate._s.Year) &&	IS_RTC_MONTH(bckDate._s.Month) && IS_RTC_DATE(bckDate._s.Date) && bckDate._s.WeekDay < 7) {
		hrtc.DateToUpdate.Year = bckDate._s.Year;
		hrtc.DateToUpdate.Month = bckDate._s.Month;
		hrtc.DateToUpdate.Date = bckDate._s.Date;
		hrtc.DateToUpdate.WeekDay = bckDate._s.WeekDay;
	}
}

void force_rtc_bck_sync(void) {
	RTC_DateTypeDef_u bckDate, coreDate;
	bckDate.raw = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) & 0xFFFF) | (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7) << 16);

	// Caution! This function increment the day of the date depending of timer counter.
	HAL_RTC_GetDate(&hrtc, &coreDate._s, RTC_FORMAT_BIN);
	// RTC is in un-sync condition (date anterior to one in SRAM), load from SRAM
	if (date_is_anterior(&coreDate._s, &bckDate._s)) {
		HAL_RTC_SetDate(&hrtc, &bckDate._s, RTC_FORMAT_BIN);
	}
	// Nominal condition
	else if (coreDate.raw != bckDate.raw) {
		// Sync
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, coreDate.raw & 0xFFFF);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, coreDate.raw >> 16);
	}

	date_sync_counter = uptime_ms();
}

void check_rtc_bck_sync(void) {
	// sync check every 1m
	if (uptime_ms() - date_sync_counter > 60000)
		force_rtc_bck_sync();
}

void i2cs_fill_buffer_RTC_date(uint8_t* const date_buff, const volatile RTC_DateTypeDef* const date_s) {
	if (date_s == NULL || date_buff == NULL)
		return;

	date_buff[0] = date_s->Year;
	date_buff[1] = date_s->Month;
	date_buff[2] = date_s->Date;
	date_buff[3] = date_s->WeekDay;
}

void i2cs_fill_buffer_RTC_time(uint8_t* const time_buff, const volatile RTC_TimeTypeDef* const time_s) {
	if (time_s == NULL || time_buff == NULL)
		return;

	time_buff[0] = time_s->Hours;
	time_buff[1] = time_s->Minutes;
	time_buff[2] = time_s->Seconds;
}

void i2cs_RTC_date_from_buffer(volatile RTC_DateTypeDef* const date_s, const uint8_t* const date_buff) {
	if (date_s == NULL || date_buff == NULL)
		return;

	date_s->Year = date_buff[0] <= 99 ? date_buff[0] : 99;
	date_s->Month = (date_buff[1] > 0 && date_buff[1] <= 12) ? date_buff[1] : 12;
	date_s->Date = (date_buff[2] > 0 && date_buff[2] <= 31) ? date_buff[2] : 31;
	//data_s.WeekDay - this element is automatically recomputed
}

void i2cs_RTC_time_from_buffer(volatile RTC_TimeTypeDef* const time_s, const uint8_t* const time_buff) {
	if (time_s == NULL || time_buff == NULL)
		return;

	time_s->Hours = time_buff[0] <= 23 ? time_buff[0] : 23;
	time_s->Minutes = time_buff[1] <= 59 ? time_buff[1] : 59;
	time_s->Seconds = time_buff[2] <= 59 ? time_buff[2] : 59;
}

inline uint32_t rtc_run_alarm(void) {
	RTC_AlarmTypeDef alarm_s;

	alarm_s.Alarm = RTC_ALARM_A;
	alarm_s.AlarmTime.Hours = rtc_alarm_time._s.Hours;
	alarm_s.AlarmTime.Minutes = rtc_alarm_time._s.Minutes;
	alarm_s.AlarmTime.Seconds = rtc_alarm_time._s.Seconds;
	return (uint32_t)HAL_RTC_SetAlarm_IT(&hrtc, &alarm_s, RTC_FORMAT_BIN);
}

inline uint32_t rtc_stop_alarm(void) {
	return (uint32_t)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
}
