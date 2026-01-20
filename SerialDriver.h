#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"
#include "circular_buffer.h"

#define SERIAL_DRIVER_TX_BUFFER_SIZE 256
#define SERIAL_DRIVER_RX_BUFFER_SIZE 256

typedef struct SerialDriver {
  uint8_t *membase;
  status_t loopback_mode;
  status_t discrete_mode;
  circular_buffer_t tx_cb;
  circular_buffer_t rx_cb;
  uint8_t tx_storage[SERIAL_DRIVER_TX_BUFFER_SIZE];
  uint8_t rx_storage[SERIAL_DRIVER_RX_BUFFER_SIZE];
} SerialDriver;

void serial_driver_init(SerialDriver *driver);
uint32_t serial_driver_write(SerialDriver *driver, const uint8_t *buffer);
uint32_t serial_driver_read(SerialDriver *driver, uint8_t *buffer,
                            uint32_t length);
void serial_driver_close(SerialDriver *driver);
void serial_driver_set_discrete(SerialDriver *driver, status_t mode);
void serial_driver_set_loopback(SerialDriver *driver, status_t mode);
bool serial_driver_loopback_enabled(const SerialDriver *driver);
bool serial_driver_discrete_enabled(const SerialDriver *driver);

#endif
