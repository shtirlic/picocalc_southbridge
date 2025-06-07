#include "stm32f1xx_hal.h"


#ifndef I2CS_H_
#define I2CS_H_

#define I2CS_REARM_TIMEOUT	500


enum i2cs_state {
	//I2CS_STATE_HALT,
	I2CS_STATE_IDLE,
	I2CS_STATE_REG_REQUEST,
	I2CS_STATE_REG_ANSWER
};

extern enum i2cs_state i2cs_state;
extern uint32_t i2cs_rearm_counter;

#endif /* I2CS_H_ */
