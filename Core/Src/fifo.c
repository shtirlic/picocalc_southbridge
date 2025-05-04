#include "fifo.h"


static struct fifo_item fifo[FIFO_SIZE] = {0};
static uint8_t count = 0;
static volatile uint8_t read_idx = 0;
static volatile uint8_t write_idx = 0;


inline uint8_t fifo_count(void) {
	return count;
}

void fifo_flush(void) {
	write_idx = 0;
	read_idx = 0;
	count = 0;
}

uint8_t fifo_enqueue(const struct fifo_item item) {
	if (count >= FIFO_SIZE)
		return 0;

	fifo[write_idx].state = item.state;
	fifo[write_idx].key = item.key;

	write_idx++;
	write_idx %= FIFO_SIZE;
	++count;

	return 1;
}

void fifo_enqueue_force(const struct fifo_item item) {
	if (fifo_enqueue(item))
		return;

	fifo[write_idx].state = item.state;
	fifo[write_idx].key = item.key;
	write_idx++;
	write_idx %= FIFO_SIZE;

	read_idx++;
	read_idx %= FIFO_SIZE;
}

void fifo_dequeue(struct fifo_item* const outItem) {
	if (outItem == NULL)
		return;

	if (count == 0)
		return;

	outItem->state = fifo[read_idx].state;
	outItem->key = fifo[read_idx].key;
	read_idx++;
	read_idx %= FIFO_SIZE;
	--count;
}
