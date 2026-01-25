#include "SerialDriver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static uint32_t pull_from_rx_cache(SerialDriver* driver, uint8_t* buffer, uint32_t length, uint32_t total_decoded) {
  uint8_t cached = 0;
  while (total_decoded < length && cb_pop(&driver->rx_cb, &cached)) {
    buffer[total_decoded++] = cached;
  }
  return total_decoded;
}

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

void serial_driver_init(SerialDriver* driver) {
  driver->discrete_mode = MODE_OFF;
  driver->loopback_mode = MODE_OFF;
  driver->UARTbase = (uint8_t*)(uintptr_t)SERIAL_DRIVER_UART_BASE;

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

uint32_t serial_driver_write(SerialDriver* driver, const uint8_t* buffer, size_t length) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* thr = port + SERIAL_PORT_OFFSET_THR;
  // const volatile uint8_t* lsr = port + SERIAL_PORT_OFFSET_LSR;
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

uint32_t serial_driver_read(SerialDriver* driver, uint8_t* buffer, uint32_t length) {
  volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  volatile uint8_t* rbr = port + SERIAL_PORT_OFFSET_RBR;
  // const volatile uint8_t* lsr = port + SERIAL_PORT_OFFSET_LSR;
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

bool serial_driver_loopback_enabled(const SerialDriver* driver) { return driver->loopback_mode == MODE_ON; }

bool serial_driver_discrete_enabled(const SerialDriver* driver) { return driver->discrete_mode == MODE_ON; }

bool serial_driver_data_available(const SerialDriver* driver) { return !cb_is_empty(&driver->rx_cb); }

bool serial_driver_transmitter_empty(const SerialDriver* driver) {
  const volatile uint8_t* port = (volatile uint8_t*)driver->UARTbase;
  const volatile uint8_t* lsr = port + SERIAL_PORT_OFFSET_LSR;
  return ((*lsr & SERIAL_PORT_STATUS_THR_EMPTY) != 0);
}

bool wait_for_thr_empty(SerialDriver* driver, uint32_t timeout_ms) {
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
