/**
 * @file circular_buffer.h
 * @brief Fixed-size circular buffer (ring buffer) API for bytes.
 *
 * The caller provides storage; this module does not allocate or free memory.
 */
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @struct circular_buffer_t
 * @brief A fixed-size circular buffer implementation for efficient queue operations.
 *
 * @details
 * This structure implements a circular buffer (ring buffer) that allows efficient
 * enqueue and dequeue operations without data shifting. It uses a fixed capacity
 * and wraps around when reaching the end of the allocated memory.
 *
 * @var circular_buffer_t::data
 *     Pointer to the caller-provided storage buffer.
 *
 * @var circular_buffer_t::capacity
 *     Maximum number of elements the buffer can hold. Power-of-two capacities
 *     enable faster index wrapping, but any capacity is supported.
 *
 * @var circular_buffer_t::head
 *     Index pointing to the next write position. Elements are written at this position.
 *
 * @var circular_buffer_t::tail
 *     Index pointing to the next read position. Elements are read from this position.
 *
 * @var circular_buffer_t::size
 *     Current number of valid elements stored in the buffer (0 to capacity).
 *
 * @var circular_buffer_t::mask
 *     Bitmask used for efficient modulo operations (capacity - 1) when
 *     capacity is a power of 2. When not a power of 2, the mask is 0 and
 *     wrapping uses a conditional branch instead.
 */
typedef struct {
  uint32_t* data;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t size;
  size_t mask;
} circular_buffer_t;

/**
 * @brief Initializes a circular buffer.
 *
 * @param cb Pointer to the circular buffer structure to initialize.
 * @param storage Pointer to the byte array that will serve as the buffer's storage.
 * @param capacity The total number of elements the storage array can hold.
 *
 * @note The caller is responsible for ensuring that the storage array has
 *       sufficient memory allocated for the specified capacity. If capacity
 *       is non-zero, storage must be non-NULL. Power-of-two capacities yield
 *       faster wrap behavior.
 */
void cb_init(circular_buffer_t* cb, uint32_t* storage, size_t capacity);

/**
 * @brief Resets the circular buffer to its initial empty state.
 *
 * Clears all data in the circular buffer and resets internal pointers
 * (head and tail) to their initial positions. After this operation,
 * the buffer will be empty and ready to accept new data.
 *
 * @param cb Pointer to the circular_buffer_t structure to reset.
 *           Must not be NULL.
 *
 * @return void
 *
 * @note This function does not deallocate memory; it only resets the
 *       buffer state. The buffer structure and its allocated memory
 *       remain valid after the reset. The storage contents are not cleared.
 */
void cb_reset(circular_buffer_t* cb);

/**
 * @brief Checks if the circular buffer is empty.
 *
 * @param cb Pointer to the circular buffer structure to check.
 *
 * @return true if the circular buffer contains no elements, false otherwise.
 */
bool cb_is_empty(const circular_buffer_t* cb);
/**
 * @brief Checks if the circular buffer is full.
 * @param cb Pointer to the circular buffer structure.
 * @return true if the circular buffer is full, false otherwise.
 */
bool cb_is_full(const circular_buffer_t* cb);

/**
 * @brief Gets the current number of elements in the circular buffer.
 *
 * @param cb Pointer to the circular buffer structure.
 *
 * @return The number of elements currently stored in the circular buffer.
 */
size_t cb_size(const circular_buffer_t* cb);

/**
 * @brief Gets the maximum capacity of the circular buffer.
 *
 * @param cb Pointer to the circular buffer structure.
 *
 * @return The maximum number of elements the circular buffer can hold.
 */
size_t cb_capacity(const circular_buffer_t* cb);

/**
 * @brief Pushes a value onto the circular buffer.
 * @param cb Pointer to the circular buffer structure.
 * @param value The 8-bit unsigned integer value to push onto the buffer.
 * @return true if the value was successfully pushed, false if the buffer is full
 *         or has zero capacity.
 */
bool cb_push(circular_buffer_t* cb, uint32_t value);

/**
 * @brief Removes and retrieves an element from the circular buffer.
 *
 * Pops the oldest element from the circular buffer and stores it in the
 * provided value pointer. The element is removed from the buffer.
 *
 * @param[in] cb Pointer to the circular buffer structure.
 * @param[out] value Pointer to where the popped value will be stored.
 *                  May be NULL to discard the popped value.
 *
 * @return true if an element was successfully popped from the buffer.
 * @return false if the buffer is empty or if invalid parameters were provided.
 */
bool cb_pop(circular_buffer_t* cb, uint32_t* value);

/**
 * @brief Peeks at the front element of the circular buffer without removing it.
 *
 * Retrieves the value at the front of the circular buffer without modifying
 * the buffer state. The element remains in the buffer after this operation.
 *
 * @param[in] cb Pointer to the circular buffer structure.
 * @param[out] value Pointer to a uint8_t variable where the peeked value will be stored.
 *
 * @return true if the peek operation was successful and a value was retrieved,
 *         false if the circular buffer is empty or if invalid parameters were provided.
 */
bool cb_peek(const circular_buffer_t* cb, uint32_t* value);

#ifdef __cplusplus
}
#endif

#endif
