#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

// Register Offsets
enum {
  SERIAL_PORT_OFFSET_RBR = 0x0,  // Receiver Buffer Register (read)
  SERIAL_PORT_OFFSET_THR = 0x0,  // Transmitter Holding Register (write)
  SERIAL_PORT_OFFSET_IER = 0x1,  // Interrupt Enable Register
  SERIAL_PORT_OFFSET_IIR = 0x2,  // Interrupt Identification Register (read)
  SERIAL_PORT_OFFSET_FCR = 0x2,  // FIFO Control Register (write)
  SERIAL_PORT_OFFSET_LCR = 0x3,  // Line Control Register
  SERIAL_PORT_OFFSET_MCR = 0x4,  // Modem Control Register
  SERIAL_PORT_OFFSET_LSR = 0x5,  // Line Status Register
  SERIAL_PORT_OFFSET_MSR = 0x6,  // Modem Status Register
  SERIAL_PORT_OFFSET_SCR = 0x7,  // Scratch Register
  SERIAL_PORT_OFFSET_DLL = 0x0,  // Divisor Latch Low (when DLAB=1)
  SERIAL_PORT_OFFSET_DLH = 0x1   // Divisor Latch High (when DLAB=1)
};

// Legacy aliases for backward compatibility
enum { SERIAL_PORT_OFFSET_DATA = 0x0, SERIAL_PORT_OFFSET_STATUS = 0x5 };

// Legacy constants for backward compatibility
enum { SERIAL_PORT_STATUS_DATA_READY = 0x01, SERIAL_PORT_STATUS_THR_EMPTY = 0x20 };

enum { FLAG = 0x7E, ESCAPE = 0x7D, ESCAPE_XOR = 0x20 };

typedef enum { MODE_OFF = 0, MODE_ON = 1 } status_t;

#endif  // TYPES_H
