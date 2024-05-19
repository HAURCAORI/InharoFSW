#ifndef _TELEMETRY_HANDLER_H_
#define _TELEMETRY_HANDLER_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TLE_Circular_Buffer circular_buffer
#define TLE_Circular_Buffer_Size 5

#define cbResult 		uint8_t
#define cb_ok 			1<<0
#define cb_error		1<<1
#define cb_null 1<<2

typedef struct circular_buffer {
	void* buffer;
	void* buffer_end;
	size_t capacity;
	size_t count;
	size_t sz;
	void* head;
	void* tail;
}circular_buffer;

static cbResult cb_init(circular_buffer* cb, size_t capacity, size_t sz) {
	cb->buffer = malloc(capacity*sz);
	if(cb->buffer == NULL) {
		return cb_error;
	}
	cb->buffer_end = (char *)cb->buffer + capacity * sz;
	cb->capacity = capacity;
	cb->sz = sz;
	cb->head = cb->buffer;
	cb->tail = cb->buffer;
	return cb_ok;
}

static void cb_free(circular_buffer* cb) {
	free(cb->buffer);
}

static void cb_push(circular_buffer *cb, const void *item)
{
    if(cb->count == cb->capacity){
    	cb->tail = (char*) cb->tail + cb->sz;
    	if(cb->tail == cb->buffer_end)
    		cb->tail = cb->buffer;
    	cb->count--;
    }
    memcpy(cb->head, item, cb->sz);
    cb->head = (char*)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
}

static cbResult cb_pop(circular_buffer *cb, void *item)
{
    if(cb->count == 0){ return cb_null; }
    memcpy(item, cb->tail, cb->sz);
    cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;

    return cb_ok;
}

static cbResult cb_value(circular_buffer *cb, void *item) {
	if(cb->count == 0) {return cb_null; }
	memcpy(item,cb->tail, cb->sz);
	return cb_ok;
}

static size_t cb_count(circular_buffer *cb) {
	return cb->count;
}

#endif
