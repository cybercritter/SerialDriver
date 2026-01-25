# Serial Driver

## SonarCloud Status
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)
[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=bugs)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=code_smells)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)
[![wakatime](https://wakatime.com/badge/user/fc6ec941-b731-4418-934e-aa341fe867eb/project/4a49c825-9c8a-47ff-b093-db5086d37cd9.svg)](https://wakatime.com/badge/user/fc6ec941-b731-4418-934e-aa341fe867eb/project/4a49c825-9c8a-47ff-b093-db5086d37cd9)

## Overview
SerialDriver is a C library that implements a memory-mapped UART driver with
HDLC-style byte-stuffing framing (FLAG=0x7E, ESCAPE=0x7D). It uses fixed-size
circular buffers for transmit/receive queues and provides a small API for
initialization, read/write, and modem control flags.

## Features
- Memory-mapped UART access with register offsets in `inc/common_types.h`.
- HDLC-style byte stuffing on transmit and unstuffing on receive.
- Fixed-size tx/rx circular buffers (256 bytes each).
- Polling-based transmitter drain with timeout support.
- Discrete and loopback mode control via MCR bits.

## Project Layout
- `inc/SerialDriver.h`: Public API and driver struct.
- `inc/circular_buffer.h`: Circular buffer interface used by the driver.
- `inc/common_types.h`: UART register offsets, flags, and framing constants.
- `src/SerialDriver.c`: Driver implementation and framing logic.
- `src/circular_buffer.c`: Circular buffer implementation.
- `demo/demo_serial_driver.c`: Minimal usage example.
- `Makefile`: Build for static library and demo binary.

## Build
```sh
make
```

Artifacts:
- `lib/libserialdriver.a`: Static library.
- `bin/demo_serial_driver`: Demo executable.

Clean:
```sh
make clean
```

## Quick Start (Demo)
```c
#include "SerialDriver.h"

int main(void) {
  SerialDriver driver;
  serial_driver_init(&driver);

  serial_driver_set_discrete(&driver, MODE_ON);
  serial_driver_set_loopback(&driver, MODE_ON);

  const char write_data[] = "Hello, Serial Port!";
  serial_driver_write(&driver, (const uint8_t *)write_data, sizeof(write_data));

  uint8_t read_buffer[128];
  serial_driver_read(&driver, read_buffer, sizeof(read_buffer));

  serial_driver_close(&driver);
  return 0;
}
```

## API Reference
### Serial driver
- `void serial_driver_init(SerialDriver* driver);`
  - Initializes the UART registers and internal tx/rx buffers. Uses `malloc`
    for a simulated UART base; replace with the actual base address for hardware.
- `uint32_t serial_driver_write(SerialDriver* driver, const uint8_t* buffer, size_t length);`
  - Encodes input with byte-stuffing and writes encoded bytes to the UART.
  - Returns the number of encoded bytes written (can be larger than `length`).
- `uint32_t serial_driver_read(SerialDriver* driver, uint8_t* buffer, uint32_t length);`
  - Reads raw bytes, decodes escapes, and returns decoded bytes up to `length`.
- `void serial_driver_close(SerialDriver* driver);`
  - Resets modem control, clears buffers, and frees the UART base.
- `void serial_driver_set_discrete(SerialDriver* driver, status_t mode);`
  - Sets or clears MCR bit 1.
- `void serial_driver_set_loopback(SerialDriver* driver, status_t mode);`
  - Sets or clears MCR bit 4.
- `bool serial_driver_loopback_enabled(const SerialDriver* driver);`
- `bool serial_driver_discrete_enabled(const SerialDriver* driver);`
- `bool serial_driver_data_available(const SerialDriver* driver);`
  - Returns true if the rx circular buffer is not empty.
- `bool wait_for_thr_empty(SerialDriver* driver, uint32_t timeout_ms);`
  - Polls LSR for `SERIAL_PORT_STATUS_THR_EMPTY` with a millisecond timeout.

### Circular buffer
The circular buffer stores bytes and supports constant-time push/pop.

- `void cb_init(circular_buffer_t* cb, uint8_t* storage, size_t capacity);`
- `void cb_reset(circular_buffer_t* cb);`
- `bool cb_is_empty(const circular_buffer_t* cb);`
- `bool cb_is_full(const circular_buffer_t* cb);`
- `size_t cb_size(const circular_buffer_t* cb);`
- `size_t cb_capacity(const circular_buffer_t* cb);`
- `bool cb_push(circular_buffer_t* cb, uint8_t value);`
- `bool cb_pop(circular_buffer_t* cb, uint8_t* value);`
- `bool cb_peek(const circular_buffer_t* cb, uint8_t* value);`

## Framing Details
HDLC-style byte stuffing protects the special bytes `FLAG` (0x7E) and `ESCAPE`
(0x7D). On transmit, these bytes are escaped by prefixing `ESCAPE` and XORing
with `ESCAPE_XOR` (0x20). On receive, the inverse operation is applied.

Example (hex):
- Input: `7E 01 7D 02`
- Encoded: `7D 5E 01 7D 5D 02`

## Examples
### Byte stuffing and write
```c
const uint8_t payload[] = {0x7E, 0x01, 0x7D, 0x02};
uint32_t encoded_written = serial_driver_write(&driver, payload, sizeof(payload));
```

### Read decoded bytes
```c
uint8_t rx[64];
uint32_t got = serial_driver_read(&driver, rx, sizeof(rx));
if (got > 0) {
  // rx contains decoded payload bytes.
}
```

### Circular buffer usage
```c
uint8_t storage[8];
circular_buffer_t cb;
cb_init(&cb, storage, sizeof(storage));
cb_push(&cb, 0x11);
cb_push(&cb, 0x22);
uint8_t out = 0;
cb_pop(&cb, &out);
```

### UART register access
```c
volatile uint8_t* port = (volatile uint8_t*)driver.UARTbase;
uint8_t line_status = port[SERIAL_PORT_OFFSET_LSR];
```

## Nucleo-F411RE Integration
### Target MCU
- STM32F411RE (ARM Cortex-M4F)
- On-board ST-LINK V2-1 provides USB virtual COM port (VCP)

### Hardware Interface
- VCP is wired to USART2 on the Nucleo-F411RE.
- TX: PA2 (USART2_TX)
- RX: PA3 (USART2_RX)
- Alternate function: AF7 for USART2 on PA2/PA3

### STM32F411 (Nucleo-F411RE) Example
```c
// Replace malloc-based UART base with the MCU's USART2 base.
// Requires STM32 CMSIS device headers for USART2 definition.
#include "stm32f4xx.h"

## API Reference
### Serial driver
- `void serial_driver_init(SerialDriver* driver);`
  - Initializes the UART registers and internal tx/rx buffers. Uses `malloc`
    for a simulated UART base; replace with the actual base address for hardware.
- `uint32_t serial_driver_write(SerialDriver* driver, const uint8_t* buffer, size_t length);`
  - Encodes input with byte-stuffing and writes encoded bytes to the UART.
  - Returns the number of encoded bytes written (can be larger than `length`).
- `uint32_t serial_driver_read(SerialDriver* driver, uint8_t* buffer, uint32_t length);`
  - Reads raw bytes, decodes escapes, and returns decoded bytes up to `length`.
- `void serial_driver_close(SerialDriver* driver);`
  - Resets modem control, clears buffers, and frees the UART base.
- `void serial_driver_set_discrete(SerialDriver* driver, status_t mode);`
  - Sets or clears MCR bit 1.
- `void serial_driver_set_loopback(SerialDriver* driver, status_t mode);`
  - Sets or clears MCR bit 4.
- `bool serial_driver_loopback_enabled(const SerialDriver* driver);`
- `bool serial_driver_discrete_enabled(const SerialDriver* driver);`
- `bool serial_driver_data_available(const SerialDriver* driver);`
  - Returns true if the rx circular buffer is not empty.
- `bool wait_for_thr_empty(SerialDriver* driver, uint32_t timeout_ms);`
  - Polls LSR for `SERIAL_PORT_STATUS_THR_EMPTY` with a millisecond timeout.

### Circular buffer
The circular buffer stores bytes and supports constant-time push/pop.

- `void cb_init(circular_buffer_t* cb, uint8_t* storage, size_t capacity);`
- `void cb_reset(circular_buffer_t* cb);`
- `bool cb_is_empty(const circular_buffer_t* cb);`
- `bool cb_is_full(const circular_buffer_t* cb);`
- `size_t cb_size(const circular_buffer_t* cb);`
- `size_t cb_capacity(const circular_buffer_t* cb);`
- `bool cb_push(circular_buffer_t* cb, uint8_t value);`
- `bool cb_pop(circular_buffer_t* cb, uint8_t* value);`
- `bool cb_peek(const circular_buffer_t* cb, uint8_t* value);`

## Framing Details
HDLC-style byte stuffing protects the special bytes `FLAG` (0x7E) and `ESCAPE`
(0x7D). On transmit, these bytes are escaped by prefixing `ESCAPE` and XORing
with `ESCAPE_XOR` (0x20). On receive, the inverse operation is applied.

Example (hex):
- Input: `7E 01 7D 02`
- Encoded: `7D 5E 01 7D 5D 02`

## Examples
### Byte stuffing and write
```c
const uint8_t payload[] = {0x7E, 0x01, 0x7D, 0x02};
uint32_t encoded_written = serial_driver_write(&driver, payload, sizeof(payload));
```

### Read decoded bytes
```c
uint8_t rx[64];
uint32_t got = serial_driver_read(&driver, rx, sizeof(rx));
if (got > 0) {
  // rx contains decoded payload bytes.
}
```

### Circular buffer usage
```c
uint8_t storage[8];
circular_buffer_t cb;
cb_init(&cb, storage, sizeof(storage));
cb_push(&cb, 0x11);
cb_push(&cb, 0x22);
uint8_t out = 0;
cb_pop(&cb, &out);
```

### UART register access
```c
volatile uint8_t* port = (volatile uint8_t*)driver.UARTbase;
uint8_t line_status = port[SERIAL_PORT_OFFSET_LSR];
```

## Nucleo-F411RE Integration
### Target MCU
- STM32F411RE (ARM Cortex-M4F)
- On-board ST-LINK V2-1 provides USB virtual COM port (VCP)

### Hardware Interface
- VCP is wired to USART2 on the Nucleo-F411RE.
- TX: PA2 (USART2_TX)
- RX: PA3 (USART2_RX)
- Alternate function: AF7 for USART2 on PA2/PA3

### STM32F411 (Nucleo-F411RE) Example
```c
// Replace malloc-based UART base with the MCU's USART2 base.
// Requires STM32 CMSIS device headers for USART2 definition.
#include "stm32f4xx.h"

SerialDriver driver;
serial_driver_init(&driver);
driver.UARTbase = (uint8_t*)USART2;

// Ensure RCC clocking and GPIO alternate-function setup are enabled for USART2
// before use (e.g., enable RCC_APB1ENR_USART2EN and configure GPIOA pins to AF7).
```

### GPIO/Clock Setup Reminder
- Enable GPIOA and USART2 clocks (RCC_AHB1ENR for GPIOA, RCC_APB1ENR for USART2).
- Configure PA2/PA3 to AF7, push-pull, high-speed, and appropriate pull-ups.
- Set USART2 baud rate and frame format consistent with your host settings.

## Integration Notes
- The demo uses `malloc` for a fake UART base. On STM Nucleo-F411RE, replace
  allocation with the actual USART base address from the STM32F411 reference
  manual (e.g., USART2 for the ST-LINK VCP).
- Polling is used for transmit readiness via `wait_for_thr_empty`.
- Buffer sizes are fixed at 256 bytes each; update
  `SERIAL_DRIVER_TX_BUFFER_SIZE` and `SERIAL_DRIVER_RX_BUFFER_SIZE` if needed.

## License
Copyright (c) Cybercritter Software
driver.UARTbase = (uint8_t*)USART2;

// Ensure RCC clocking and GPIO alternate-function setup are enabled for USART2
// before use (e.g., enable RCC_APB1ENR_USART2EN and configure GPIOA pins to AF7).
```

### GPIO/Clock Setup Reminder
- Enable GPIOA and USART2 clocks (RCC_AHB1ENR for GPIOA, RCC_APB1ENR for USART2).
- Configure PA2/PA3 to AF7, push-pull, high-speed, and appropriate pull-ups.
- Set USART2 baud rate and frame format consistent with your host settings.

## Integration Notes
- The demo uses `malloc` for a fake UART base. On STM Nucleo-F411RE, replace
  allocation with the actual USART base address from the STM32F411 reference
  manual (e.g., USART2 for the ST-LINK VCP).
- Polling is used for transmit readiness via `wait_for_thr_empty`.
- Buffer sizes are fixed at 256 bytes each; update
  `SERIAL_DRIVER_TX_BUFFER_SIZE` and `SERIAL_DRIVER_RX_BUFFER_SIZE` if needed.

## License
Copyright (c) Cybercritter Software
