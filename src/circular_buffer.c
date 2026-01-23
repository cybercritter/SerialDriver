#include "circular_buffer.h"

static inline bool cb_is_power_of_two(size_t value) { return value != 0 && (value & (value - 1)) == 0; }

void cb_init(circular_buffer_t* cb, uint8_t* storage, size_t capacity) {
  cb->data = storage;
  cb->capacity = capacity;
  cb->head = 0;
  cb->tail = 0;
  cb->size = 0;
  cb->mask = cb_is_power_of_two(capacity) ? (capacity - 1) : 0;
}

void cb_reset(circular_buffer_t* cb) {
  cb->head = 0;
  cb->tail = 0;
  cb->size = 0;
}

bool cb_is_empty(const circular_buffer_t* cb) { return cb->size == 0; }

bool cb_is_full(const circular_buffer_t* cb) { return cb->size == cb->capacity; }

size_t cb_size(const circular_buffer_t* cb) { return cb->size; }

size_t cb_capacity(const circular_buffer_t* cb) { return cb->capacity; }

bool cb_push(circular_buffer_t* cb, uint8_t value) {
  if (cb->size == cb->capacity || cb->capacity == 0) {
    return false;
  }

  cb->data[cb->head] = value;
  if (cb->mask) {
    cb->head = (cb->head + 1) & cb->mask;
  } else if (++cb->head == cb->capacity) {
    cb->head = 0;
  }
  ++cb->size;
  return true;
}

bool cb_pop(circular_buffer_t* cb, uint8_t* value) {
  if (cb->size == 0) {
    return false;
  }

  if (value) {
    *value = cb->data[cb->tail];
  }
  if (cb->mask) {
    cb->tail = (cb->tail + 1) & cb->mask;
  } else if (++cb->tail == cb->capacity) {
    cb->tail = 0;
  }
  --cb->size;
  return true;
}

bool cb_peek(const circular_buffer_t* cb, uint8_t* value) {
  if (cb->size == 0 || !value) {
    return false;
  }
  *value = cb->data[cb->tail];
  return true;
}
