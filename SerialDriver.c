#include "SerialDriver.h"
#include "types.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void encode_buffer(const uint8_t *input, uint32_t length,
                          uint8_t *output, uint32_t *out_length) {
  const uint8_t *in = input;
  const uint8_t *end = in + length;
  uint8_t *out = output;
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

static void decode_buffer(const uint8_t *input, uint32_t length,
                          uint8_t *output, uint32_t *out_length) {
  const uint8_t *in = input;
  const uint8_t *end = in + length;
  uint8_t *out = output;
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

void serial_driver_init(SerialDriver *driver) {

  printf("Serial Driver Init\n");
  driver->discrete_mode = MODE_OFF;
  driver->loopback_mode = MODE_OFF;

  // SERIAL_PORT_BASE = (uint32_t*) alloc(256); // Example allocation, replace with actual port base address as needed
  
  driver->membase = (uint8_t *)(uintptr_t)malloc(256*(sizeof(uint8_t)));
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;

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

uint32_t serial_driver_write(SerialDriver *driver, const uint8_t *buffer) {
  printf("Serial Driver Write\n");
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;
  volatile uint8_t *thr = port + SERIAL_PORT_OFFSET_THR;
  volatile uint8_t *lsr = port + SERIAL_PORT_OFFSET_LSR;
  uint32_t written = 0;

  uint32_t input_length = (uint32_t)strlen((const char *)buffer);
  if (input_length == 0) {
    return 0;
  }

  {
    enum { kChunk = 64 };
    uint8_t encoded[kChunk * 2];
    uint32_t offset = 0;
    while (offset < input_length) {
      uint32_t chunk = input_length - offset;
      if (chunk > kChunk) {
        chunk = kChunk;
      }
      uint32_t encoded_length = 0;
      encode_buffer(buffer + offset, chunk, encoded, &encoded_length);
      offset += chunk;

      {
        const uint8_t *p = encoded;
        const uint8_t *end = p + encoded_length;
        while (p < end) {
          while (cb_is_full(&driver->tx_cb)) {
            while ((*lsr & SERIAL_PORT_STATUS_THR_EMPTY) == 0) {
            }
            {
              uint8_t out_byte = 0;
              cb_pop(&driver->tx_cb, &out_byte);
              *thr = out_byte;
              ++written;
            }
          }
          cb_push(&driver->tx_cb, *p++);
        }

        while (!cb_is_empty(&driver->tx_cb)) {
          while ((*lsr & SERIAL_PORT_STATUS_THR_EMPTY) == 0) {
          }
          {
            uint8_t out_byte = 0;
            cb_pop(&driver->tx_cb, &out_byte);
            *thr = out_byte;
            ++written;
          }
        }
      }
    }
  }

  return written;
}

uint32_t serial_driver_read(SerialDriver *driver, uint8_t *buffer,
                            uint32_t length) {

  printf("Serial Driver Read\n");
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;
  volatile uint8_t *rbr = port + SERIAL_PORT_OFFSET_RBR;
  volatile uint8_t *lsr = port + SERIAL_PORT_OFFSET_LSR;
  if (length == 0) {
    return 0;
  }

  {
    enum { kChunk = 64 };
    uint8_t raw[kChunk + 1];
    uint8_t decoded[kChunk];
    uint32_t total_decoded = 0;
    uint8_t carry_escape = 0;

    while (total_decoded < length) {
      uint8_t cached = 0;
      while (total_decoded < length && cb_pop(&driver->rx_cb, &cached)) {
        buffer[total_decoded++] = cached;
      }
      if (total_decoded >= length) {
        break;
      }

      {
        uint32_t raw_len = 0;
        if (carry_escape) {
          raw[raw_len++] = ESCAPE;
          carry_escape = 0;
        }

        uint32_t decoded_in_chunk = 0;
        uint8_t escaped = 0;
        while (raw_len < kChunk && decoded_in_chunk < (length - total_decoded)) {
          while ((*lsr & SERIAL_PORT_STATUS_DATA_READY) == 0) {
          }
          {
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
        }

        if (raw_len > 0 && raw[raw_len - 1] == ESCAPE) {
          carry_escape = 1;
          --raw_len;
        }

        {
          uint32_t out_length = 0;
          decode_buffer(raw, raw_len, decoded, &out_length);
          {
            uint32_t i = 0;
            while (i < out_length) {
              cb_push(&driver->rx_cb, decoded[i]);
              ++i;
            }
          }
        }
      }
    }

    return total_decoded;
  }
}

void serial_driver_close(SerialDriver *driver) {
  printf("Serial Driver Close\n");
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;

  port[SERIAL_PORT_OFFSET_IER] = 0x00; /* disable interrupts */
  port[SERIAL_PORT_OFFSET_FCR] = 0x00; /* disable FIFO */
  port[SERIAL_PORT_OFFSET_MCR] = 0x00; /* drop modem control lines */

  driver->discrete_mode = MODE_OFF;
  driver->loopback_mode = MODE_OFF;
  cb_reset(&driver->tx_cb);
  cb_reset(&driver->rx_cb);

  free(driver->membase);
  driver->membase = NULL;
}

void serial_driver_set_discrete(SerialDriver *driver, status_t mode) {
  printf("Serial Driver Set Discrete Mode: %s\n", mode == MODE_ON ? "ON" : "OFF");
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;
  volatile uint8_t *mcr = port + SERIAL_PORT_OFFSET_MCR;
  uint8_t value = *mcr;
  if (mode) {
    value |= (uint8_t)(1u << 1);
  } else {
    value &= (uint8_t)~(1u << 1);
  }
  *mcr = value;
  driver->discrete_mode = mode;
}

void serial_driver_set_loopback(SerialDriver *driver, status_t mode) {

  printf("Serial Driver Set Loopback Mode: %s\n", mode == MODE_ON ? "ON" : "OFF");
  volatile uint8_t *port = (volatile uint8_t *)driver->membase;
  volatile uint8_t *mcr = port + SERIAL_PORT_OFFSET_MCR;
  uint8_t value = *mcr;
  if (mode) {
    value |= (uint8_t)(1u << 4);
  } else {
    value &= (uint8_t)~(1u << 4);
  }
  *mcr = value;
  driver->loopback_mode = mode;
}

bool serial_driver_loopback_enabled(const SerialDriver *driver) {
  printf("Serial Driver Loopback Enabled Check: %s\n", driver->loopback_mode == MODE_ON ? "YES" : "NO");
  return driver->loopback_mode == MODE_ON;
}

bool serial_driver_discrete_enabled(const SerialDriver *driver) {
  printf("Serial Driver Discrete Enabled Check: %s\n", driver->discrete_mode == MODE_ON ? "YES" : "NO");
  return driver->discrete_mode == MODE_ON;
}
