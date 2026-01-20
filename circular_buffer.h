#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t *data;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t size;
  size_t mask;
} circular_buffer_t;

void cb_init(circular_buffer_t *cb, uint8_t *storage, size_t capacity);
void cb_reset(circular_buffer_t *cb);
bool cb_is_empty(const circular_buffer_t *cb);
bool cb_is_full(const circular_buffer_t *cb);
size_t cb_size(const circular_buffer_t *cb);
size_t cb_capacity(const circular_buffer_t *cb);
bool cb_push(circular_buffer_t *cb, uint8_t value);
bool cb_pop(circular_buffer_t *cb, uint8_t *value);
bool cb_peek(const circular_buffer_t *cb, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif
