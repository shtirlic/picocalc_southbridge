#include "stm32f1xx_hal.h"
#include "keyboard.h"


#ifndef FIFO_H_
#define FIFO_H_

#define FIFO_SIZE	31

struct fifo_item {
	uint8_t key;
	enum key_state state;
};


uint8_t fifo_count(void);
void fifo_flush(void);
uint8_t fifo_enqueue(const struct fifo_item item);
void fifo_enqueue_force(const struct fifo_item item);
void fifo_dequeue(struct fifo_item* const outItem);

#endif /* FIFO_H_ */
