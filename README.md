# Serial Driver Readme

## SonarCloude Status
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)
 [![Bugs](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=bugs)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver) [![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=code_smells)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver) [![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=cybercritter_SerialDriver&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=cybercritter_SerialDriver)



## Project Overview
This is a C library implementing a serial communication driver with HDLC-style byte stuffing framing (FLAG=0x7E, ESCAPE=0x7D). It provides buffered I/O over memory-mapped UART registers using circular buffers for transmit/receive queues.

## Architecture
- **Core Components**: `SerialDriver` struct manages UART state, modes, and tx/rx circular buffers
- **Data Flow**: Write operations encode data with framing, queue to tx buffer, drain to UART; Read operations decode framed data from UART, cache in rx buffer
- **Key Abstractions**: Direct register access via volatile pointers, timeout-based polling for UART status

## Build & Development
- **Build Library**: Run `make` in project root to generate `serialdriverlib.a` static library
- **Clean**: `make clean` removes obj/, bin/, and library files
- **Dependencies**: GCC, standard C library; no external deps
- **Structure**: Headers in `inc/`, sources in `src/`, objects in `obj/`, output in `bin/`

## Coding Patterns
- **Initialization**: Call `serial_driver_init()` first; allocates simulated UART memory (replace malloc with actual base address for real hardware)
- **Modes**: Use `serial_driver_set_discrete()` and `serial_driver_set_loopback()` to control MCR register bits (discrete=bit1, loopback=bit4)
- **Buffering**: Tx/rx use fixed-size circular buffers (256 bytes each); check `serial_driver_data_available()` before reading
- **Framing**: Automatic byte stuffing on write, unstuffing on read; handles FLAG/ESCAPE escaping
- **Timeouts**: Polling operations use `wait_for_thr_empty()` with millisecond timeouts (e.g., 250ms for tx drain)
- **Register Access**: Volatile pointers for UART registers; offsets defined in `common_types.h` enum

## API Usage Examples
```c
SerialDriver driver;
serial_driver_init(&driver);
serial_driver_set_loopback(&driver, MODE_ON);  // Enable loopback for testing
uint32_t written = serial_driver_write(&driver, data, len);  // Encodes and queues
uint32_t read = serial_driver_read(&driver, buffer, sizeof(buffer));  // Decodes and returns
serial_driver_close(&driver);  // Cleanup
```

## Code Style
- **Formatting**: Enforced by `.clang-format`; run `clang-format -i file.c` to format
- **Naming**: `snake_case` for functions/variables, `SCREAMING_SNAKE_CASE` for constants
- **Headers**: Include guards with `#ifndef NAME_H`, extern "C" for C++ compatibility
- **Types**: Use `status_t` (MODE_OFF/MODE_ON) for boolean-like states, `uint8_t` for bytes

## Key Files
- [`inc/SerialDriver.h`](inc/SerialDriver.h): Public API and struct definitions
- [`src/SerialDriver.c`](src/SerialDriver.c): Core implementation with encoding/decoding logic
- [`inc/circular_buffer.h`](inc/circular_buffer.h): Buffer abstraction (power-of-two optimized)
- [`inc/common_types.h`](inc/common_types.h): UART register offsets and constants
- [`Makefile`](Makefile): Build configuration for static library

## Common Pitfalls
- Always initialize driver before use; UART base is malloc'd in demo code
- Respect buffer sizes; writes block when tx buffer full, reads return available data
- Framing is transparent but requires complete frames for correct decoding
- Volatile access required for real hardware; simulated in current code</content>
