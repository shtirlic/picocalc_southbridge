#include "stm32f1xx_hal.h"


#ifndef BATT_H_
#define BATT_H_


#define LOW_BAT_VAL		(20)


void show_bat_segs(void);

void low_bat(void);

void start_chg(void);
void stop_chg(void);

#endif /* BATT_H_ */
