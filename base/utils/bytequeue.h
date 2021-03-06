/*
 * bytequeue.h
 *
 *      Author: schwarzer
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct ByteQueue
{
	uint8_t* write;
	uint8_t* read;
	uint8_t* end;
	uint32_t count;
	uint32_t size;
	uint8_t* buffer;
} ByteQueue;

void BQInit(ByteQueue* bq, uint8_t* buffer, uint32_t bufferSize);

static inline uint32_t BQCount(ByteQueue* bq)
{
	return bq->count;
};

static inline uint32_t BQSize(ByteQueue* bq)
{
	return bq->size;
};


bool BQPush(ByteQueue* bq, uint8_t byte);
uint8_t BQPop(ByteQueue* bq);
static inline uint8_t BQPeek(ByteQueue* bq)
{
	return *bq->read;
};

uint32_t BQPushBytes(ByteQueue* bq, const uint8_t* data, uint32_t count);
uint32_t BQPopBytes(ByteQueue* bq, uint8_t* buf, uint32_t count);
uint32_t BQPeekBytes(ByteQueue* bq, uint8_t* buf, uint32_t count);

void BQRemove(ByteQueue* bq, uint32_t count);

#ifdef __cplusplus
}
#endif 

#endif /* QUEUE_H_ */
