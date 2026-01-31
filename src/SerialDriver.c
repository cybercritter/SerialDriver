#include "SerialDriver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static bool serial_driver_data_available(const SerialDriver* driver);
static bool wait_for_thr_empty(SerialDriver* driver, uint32_t timeout_ms);

/**
 * @brief Encodes a buffer of data using a specified encoding scheme.
 *
 * @param input Pointer to the input buffer containing data to be encoded.
 * @param length The length of the input buffer in bytes.
 * @param output Pointer to the output buffer where encoded data will be written.
 * @param out_length Pointer to a uint32_t that will store the length of the encoded output in bytes.
 *
 * @note This function assumes that the output buffer is large enough to hold the encoded data.
 * @note The out_length parameter must point to valid memory and will be updated with the actual output length.
 */
static void encode_buffer(const uint8_t* input, uint32_t length, uint8_t* output, uint32_t* out_length) {
  const uint8_t* in = input;
  const uint8_t* end = in + length;
  uint8_t* out = output;
  uint32_t written = 0;

  while (in < end) {
    uint8_t byte = *in++;
    if (byte == FLAG || byte == ESCAPE) {
      *out++ = ESCAPE;
      ++written;
      byte = (uint8_t)(byte ^ ESCAPE_XOR);
    }
    *out++ = byte;
    ++written;
  }

  *out_length = written;
}

/**
 * @brief Decodes the input buffer and writes the result to the output buffer.
 *
 * This function processes the given input buffer of specified length, decodes its contents,
 * and stores the decoded data in the output buffer. The actual length of the decoded output
 * is written to the location pointed to by out_length.
 *
 * @param input Pointer to the input buffer containing data to decode.
 * @param length Length of the input buffer in bytes.
 * @param output Pointer to the buffer where decoded data will be stored.
 * @param out_length Pointer to a variable where the length of the decoded output will be set.
 */
static void decode_buffer(const uint8_t* input, uint32_t length, uint8_t* output, uint32_t* out_length) {
  const uint8_t* in = input;
  const uint8_t* end = in + length;
  uint8_t* out = output;
  uint32_t written = 0;
  uint8_t escaped = 0;

  while (in < end) {
    uint8_t byte = *in++;
    if (escaped) {
      *out++ = (uint8_t)(byte ^ ESCAPE_XOR);
      ++written;
      escaped = 0;
      continue;
    }
    if (byte == ESCAPE) {
      escaped = 1;
      continue;
    }
    *out++ = byte;
    ++written;
  }

  *out_length = written;
}

/**
 * @brief Flushes the transmit buffer when it is full.
 *
 * This function attempts to flush the transmit buffer associated with the given
 * SerialDriver instance. It waits until the transmit holding register (THR) is
 * available or until the specified timeout (in milliseconds) elapses.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @param thr Pointer to the volatile transmit holding register.
 * @param timeout_ms Timeout duration in milliseconds.
 * @return Number of bytes flushed, or 0 if timeout occurred.
 */
static uint32_t flush_tx_when_full(SerialDriver* driver, volatile uint8_t* thr, uint32_t timeout_ms) {
  uint32_t written = 0;
  while (cb_is_full(&driver->tx_cb)) {
    wait_for_thr_empty(driver, timeout_ms);
    uint8_t out_byte = 0;
    cb_pop(&driver->tx_cb, &out_byte);
    *thr = out_byte;
    ++written;
  }
  return written;
}

/**
 * @brief Queues encoded bytes for transmission via the serial driver.
 *
 * This function encodes the provided data and queues it for transmission by writing to the
 * transmit holding register (THR). It processes up to 'length' bytes from the input data buffer.
 *
 * @param driver Pointer to the SerialDriver instance managing the transmission.
 * @param thr Pointer to the transmit holding register (volatile, as it may change asynchronously).
 * @param data Pointer to the buffer containing the data to be encoded and queued.
 * @param length Number of bytes to process from the data buffer.
 * @return The number of bytes successfully queued for transmission.
 */
static uint32_t queue_encoded_bytes(SerialDriver* driver, volatile uint8_t* thr, const uint8_t* data, uint32_t length) {
  uint32_t written = 0;
  const uint8_t* p = data;
  const uint8_t* end = p + length;
  while (p < end) {
    written += flush_tx_when_full(driver, thr, 250);
    cb_push(&driver->tx_cb, *p++);
  }
  return written;
}

/**
 * @brief Callback function to drain the transmit buffer.
 *
 * This function is called to handle the draining of the transmit (TX) buffer for the specified
 * SerialDriver instance. It processes the data in the buffer and interacts with the provided
 * transmit holding register (THR).
 *
 * @param driver Pointer to the SerialDriver instance.
 * @param thr Pointer to the transmit holding register (volatile).
 * @return The number of bytes drained from the transmit buffer.
 */
static uint32_t drain_tx_cb(SerialDriver* driver, volatile uint8_t* thr) {
  uint32_t written = 0;
  while (!cb_is_empty(&driver->tx_cb)) {
    wait_for_thr_empty(driver, 250);

    uint8_t out_byte = 0;
    cb_pop(&driver->tx_cb, &out_byte);
    *thr = out_byte;
    ++written;
  }
  return written;
}

/**
 * @brief Pulls data from the RX cache into the provided buffer.
 *
 * This function attempts to read up to 'length' bytes from the RX cache of the specified
 * SerialDriver instance and stores the data in 'buffer'. The function may use 'total_decoded'
 * to track or limit the amount of data processed.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @param buffer Pointer to the buffer where received data will be stored.
 * @param length Maximum number of bytes to pull from the RX cache.
 * @param total_decoded Total number of bytes decoded so far (usage depends on implementation).
 * @return The number of bytes actually pulled from the RX cache.
 */
static uint32_t pull_from_rx_cache(SerialDriver* driver, uint8_t* buffer, uint32_t length, uint32_t total_decoded) {
  uint8_t cached = 0;
  while (total_decoded < length && cb_pop(&driver->rx_cb, &cached)) {
    buffer[total_decoded++] = cached;
  }
  return total_decoded;
}

/**
 * @brief Reads raw data from the specified serial port register.
 *
 * This function reads up to 'raw_limit' bytes from the receive buffer register (RBR)
 * of the serial port associated with the given driver. The read data is stored in the
 * 'raw' buffer.
 *
 * @param driver Pointer to the SerialDriver instance managing the port.
 * @param rbr Pointer to the volatile receive buffer register (RBR) of the serial port.
 * @param raw Pointer to the buffer where the raw data will be stored.
 * @param raw_limit Maximum number of bytes to read into the 'raw' buffer.
 * @return The number of bytes actually read from the port.
 */
static uint32_t read_raw_from_port(SerialDriver* driver, const volatile uint8_t* rbr, uint8_t* raw, uint32_t raw_limit,
                                   uint32_t decoded_needed, uint8_t* carry_escape) {
  uint32_t raw_len = 0;
  if (*carry_escape) {
    raw[raw_len++] = ESCAPE;
    *carry_escape = 0;
  }

  uint32_t decoded_in_chunk = 0;
  uint8_t escaped = 0;

  while (raw_len < raw_limit && decoded_in_chunk < decoded_needed) {
    wait_for_thr_empty(driver, 1000);
    uint8_t byte = *rbr;
    raw[raw_len++] = byte;
    if (escaped) {
      escaped = 0;
      ++decoded_in_chunk;
      continue;
    }
    if (byte == ESCAPE) {
      escaped = 1;
      continue;
    }
    ++decoded_in_chunk;
  }

  if (raw_len > 0 && raw[raw_len - 1] == ESCAPE) {
    *carry_escape = 1;
    --raw_len;
  }

  return raw_len;
}

/**
 * @brief Stores decoded bytes into the driver's internal buffer.
 *
 * This function takes a buffer containing decoded bytes and stores them
 * in the SerialDriver's internal buffer. It also updates the total number
 * of decoded bytes processed.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @param buffer Pointer to the buffer containing decoded bytes.
 * @param length Number of bytes to store from the buffer.
 * @param total_decoded Total number of decoded bytes processed so far.
 * @return The number of bytes successfully stored.
 */
static uint32_t store_decoded_bytes(SerialDriver* driver, uint8_t* buffer, uint32_t length, uint32_t total_decoded,
                                    const uint8_t* decoded, uint32_t decoded_length) {
  uint32_t remaining = length - total_decoded;
  uint32_t to_copy = decoded_length;
  if (to_copy > remaining) {
    to_copy = remaining;
  }
  if (to_copy > 0) {
    memcpy(buffer + total_decoded, decoded, to_copy);
    total_decoded += to_copy;
  }

  for (uint32_t i = to_copy; i < decoded_length; ++i) {
    cb_push(&driver->rx_cb, decoded[i]);
  }

  return total_decoded;
}

/**
 * @brief Initializes the SerialDriver instance.
 *
 * This function sets up the SerialDriver structure, preparing it for use.
 * It should be called before any other operations are performed on the driver.
 *
 * @param driver Pointer to the SerialDriver instance to initialize.
 */
void serial_driver_init(SerialDriver* driver) {
  driver->discrete_mode = MODE_OFF;
  driver->loopback_mode = MODE_OFF;
  // driver->UARTbase = (uint8_t*)(uintptr_t)SERIAL_DRIVER_UART_BASE;

  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;

  port[SERIAL_PORT_OFFSET_IER] = 0x00; /* disable interrupts */
  port[SERIAL_PORT_OFFSET_LCR] = 0x80; /* enable DLAB */
  port[SERIAL_PORT_OFFSET_DLL] = 0x01; /* divisor low (115200 baud) */
  port[SERIAL_PORT_OFFSET_DLH] = 0x00; /* divisor high */
  port[SERIAL_PORT_OFFSET_LCR] = 0x03; /* 8 bits, no parity, one stop */
  port[SERIAL_PORT_OFFSET_FCR] = 0xC7; /* enable/clear FIFO, 14-byte threshold */
  port[SERIAL_PORT_OFFSET_MCR] = 0x03; /* DTR/RTS */

  cb_init(&driver->tx_cb, driver->tx_storage, SERIAL_DRIVER_TX_BUFFER_SIZE);
  cb_init(&driver->rx_cb, driver->rx_storage, SERIAL_DRIVER_RX_BUFFER_SIZE);
}

/**
 * @brief Writes data to the serial driver.
 *
 * This function transmits a specified number of bytes from the provided buffer
 * through the given SerialDriver instance.
 *
 * @param driver Pointer to the SerialDriver instance to use for transmission.
 * @param buffer Pointer to the data buffer containing bytes to send.
 * @param length Number of bytes to write from the buffer.
 * @return The number of bytes successfully written.
 */
uint32_t serial_driver_write(SerialDriver* driver, const uint8_t* buffer, size_t length) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* thr = port + SERIAL_PORT_OFFSET_THR;
  uint32_t written = 0;

  if (length == 0) {
    return 0;
  }

  enum { kChunk = SERIAL_DRIVER_CHUNK_SIZE };
  uint8_t encoded[kChunk * 2];
  uint32_t offset = 0;
  while (offset < length) {
    uint32_t chunk = length - offset;
    if (chunk > kChunk) {
      chunk = kChunk;
    }
    uint32_t encoded_length = 0;
    encode_buffer(buffer + offset, chunk, encoded, &encoded_length);
    offset += chunk;

    written += queue_encoded_bytes(driver, thr, encoded, encoded_length);
    written += drain_tx_cb(driver, thr);
  }

  return written;
}

/**
 * @brief Reads data from the serial driver into the provided buffer.
 *
 * This function attempts to read up to 'length' bytes from the serial interface
 * associated with the specified SerialDriver instance. The read data is stored
 * in the memory pointed to by 'buffer'.
 *
 * @param driver Pointer to the SerialDriver instance to read from.
 * @param buffer Pointer to the buffer where the read data will be stored.
 * @param length Maximum number of bytes to read.
 * @return The actual number of bytes read, or 0 if no data is available or an error occurs.
 */
uint32_t serial_driver_read(SerialDriver* driver, uint8_t* buffer, uint32_t length) {
  const volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  const volatile uint8_t* rbr = port + SERIAL_PORT_OFFSET_RBR;
  uint32_t total_decoded = 0;
  if (length == 0) {
    return 0;
  }

  enum { kChunk = SERIAL_DRIVER_CHUNK_SIZE };
  uint8_t raw[kChunk + 1];
  uint8_t decoded[kChunk];
  uint8_t carry_escape = 0;

  while (total_decoded < length) {
    total_decoded = pull_from_rx_cache(driver, buffer, length, total_decoded);
    if (total_decoded >= length) {
      break;
    }

    uint32_t remaining = length - total_decoded;
    uint32_t raw_len = read_raw_from_port(driver, rbr, raw, kChunk, remaining, &carry_escape);
    if (raw_len == 0) {
      continue;
    }

    uint32_t out_length = 0;
    decode_buffer(raw, raw_len, decoded, &out_length);
    if (out_length == 0) {
      continue;
    }

    total_decoded = store_decoded_bytes(driver, buffer, length, total_decoded, decoded, out_length);
  }

  return total_decoded;
}

/**
 * @brief Closes the specified serial driver and releases associated resources.
 *
 * This function terminates the connection to the serial port managed by the given
 * SerialDriver instance. It ensures that any open handles or allocated memory are
 * properly released to prevent resource leaks.
 *
 * @param driver Pointer to the SerialDriver instance to be closed.
 */
void serial_driver_close(SerialDriver* driver) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;

  port[SERIAL_PORT_OFFSET_IER] = 0x00; /* disable interrupts */
  port[SERIAL_PORT_OFFSET_FCR] = 0x00; /* disable FIFO */
  port[SERIAL_PORT_OFFSET_MCR] = 0x00; /* drop modem control lines */

  driver->discrete_mode = MODE_OFF;
  driver->loopback_mode = MODE_OFF;
  cb_reset(&driver->tx_cb);
  cb_reset(&driver->rx_cb);

  /* UART base is a fixed memory-mapped address on target hardware. */
  driver->UARTbase = NULL;
}

/**
 * @brief Sets the discrete mode of the serial driver.
 *
 * This function configures the specified SerialDriver instance to operate in the given discrete mode.
 *
 * @param driver Pointer to the SerialDriver instance to configure.
 * @param mode The discrete mode to set for the driver.
 */
void serial_driver_set_discrete(SerialDriver* driver, status_t mode) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* mcr = port + SERIAL_PORT_OFFSET_MCR;
  uint8_t value = *mcr;
  if (mode) {
    value |= (uint8_t)(1u << 1);
  } else {
    value &= (uint8_t)~(1u << 1);
  }
  *mcr = value;
  driver->discrete_mode = mode;
}

/**
 * @brief Sets the loopback mode for the specified serial driver.
 *
 * This function configures the loopback mode of the given SerialDriver instance.
 * Loopback mode is typically used for testing and debugging, allowing transmitted data
 * to be received back by the driver.
 *
 * @param driver Pointer to the SerialDriver instance to configure.
 * @param mode The desired loopback mode status.
 */
void serial_driver_set_loopback(SerialDriver* driver, status_t mode) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* mcr = port + SERIAL_PORT_OFFSET_MCR;
  uint8_t value = *mcr;
  if (mode) {
    value |= (uint8_t)(1u << 4);
  } else {
    value &= (uint8_t)~(1u << 4);
  }
  *mcr = value;
  driver->loopback_mode = mode;
}

/**
 * @brief Checks if loopback mode is enabled for the given serial driver.
 *
 * This function returns true if the serial driver's loopback mode is set to MODE_ON,
 * indicating that loopback functionality is currently enabled.
 *
 * @param driver Pointer to a SerialDriver instance.
 * @return true if loopback mode is enabled, false otherwise.
 */
bool serial_driver_loopback_enabled(const SerialDriver* driver) { return driver->loopback_mode == MODE_ON; }

/**
 * @brief Checks if the discrete mode is enabled for the given serial driver.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @return true if discrete mode is enabled, false otherwise.
 */
bool serial_driver_discrete_enabled(const SerialDriver* driver) { return driver->discrete_mode == MODE_ON; }

/**
 * @brief Checks if there is data available to read from the serial driver's receive buffer.
 *
 * This function returns true if the receive circular buffer (`rx_cb`) associated with the given
 * SerialDriver instance is not empty, indicating that there is data available to be read.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @return true if data is available to read; false otherwise.
 */
bool serial_driver_data_available(const SerialDriver* driver) { return !cb_is_empty(&driver->rx_cb); }

/**
 * @brief Checks if the serial driver's transmitter is empty.
 *
 * This function determines whether the transmitter buffer of the specified
 * SerialDriver instance is empty and ready to accept new data for transmission.
 *
 * @param driver Pointer to the SerialDriver instance to check.
 * @return true if the transmitter is empty, false otherwise.
 */
bool serial_driver_transmitter_empty(const SerialDriver* driver) {
  const volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  const volatile uint8_t* lsr = port + SERIAL_PORT_OFFSET_LSR;
  return ((*lsr & SERIAL_PORT_STATUS_THR_EMPTY) != 0);
}

/**
 * @brief Waits for the Transmit Holding Register (THR) to become empty.
 *
 * This function checks if the THR of the specified SerialDriver is empty within the given timeout period.
 *
 * @param driver Pointer to the SerialDriver instance.
 * @param timeout_ms Timeout in milliseconds to wait for the THR to become empty.
 * @return true if the THR becomes empty within the timeout, false otherwise.
 */
static bool wait_for_thr_empty(SerialDriver* driver, uint32_t timeout_ms) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* lsr = port + SERIAL_PORT_OFFSET_LSR;

  uint32_t elapsed = 0;
  const uint32_t poll_interval_ms = 1;
  struct timespec poll_interval;
  poll_interval.tv_sec = poll_interval_ms / 1000u;
  poll_interval.tv_nsec = (long)(poll_interval_ms % 1000u) * 1000000L;

  while (elapsed < timeout_ms) {
    if ((*lsr & SERIAL_PORT_STATUS_THR_EMPTY) != 0) {
      return true;
    }
    nanosleep(&poll_interval, NULL);
    elapsed += poll_interval_ms;
  }

  return false;
}
