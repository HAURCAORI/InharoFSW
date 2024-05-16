#ifndef _TELEMETRY_HANDLER_H_
#define _TELEMETRY_HANDLER_H_

#define TLE_Circular_Buffer circular buffer

#define cbResult 	uint8_t
#define cb_ok 		1<<0;
#define cb_error	1<<1;

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
		return cb_error
	}
	cb->buffer_end = (char *)cb->buffer + capacity * sz;
	cb->capacity = capacity;
	cb->sz = sz;
}

static void cb_free(circular_buffer* cb) {
	free(cb->buffer);
}
/*
 * void cb_init(circular_buffer *cb, size_t capacity, size_t sz)
{
    cb->buffer = malloc(capacity * sz);
    if(cb->buffer == NULL)
        // handle error
    cb->buffer_end = (char *)cb->buffer + capacity * sz;
    cb->capacity = capacity;
    cb->count = 0;
    cb->sz = sz;
    cb->head = cb->buffer;
    cb->tail = cb->buffer;
}

void cb_free(circular_buffer *cb)
{
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

void cb_push_back(circular_buffer *cb, const void *item)
{
    if(cb->count == cb->capacity){
        // handle error
    }
    memcpy(cb->head, item, cb->sz);
    cb->head = (char*)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
}

void cb_pop_front(circular_buffer *cb, void *item)
{
    if(cb->count == 0){
        // handle error
    }
    memcpy(item, cb->tail, cb->sz);
    cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;
}
 */

#endif
