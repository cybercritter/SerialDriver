/**
 * @file common_types.h
 * @brief Common type definitions and constants for serial port driver
 * 
 * This header file defines register offsets, status flags, and type definitions
 * used throughout the serial driver implementation. It provides support for
 * standard UART/serial port register access with both normal and DLAB
 * (Divisor Latch Access Bit) modes.
 * 
 * @details
 * Register Offsets:
 * - RBR (0x0): Receiver Buffer Register - read-only, contains received data
 * - THR (0x0): Transmitter Holding Register - write-only, holds data to transmit
 * - IER (0x1): Interrupt Enable Register - controls which interrupts are enabled
 * - IIR (0x2): Interrupt Identification Register - read-only, identifies interrupt source
 * - FCR (0x2): FIFO Control Register - write-only, controls FIFO behavior
 * - LCR (0x3): Line Control Register - controls data bits, parity, stop bits, DLAB
 * - MCR (0x4): Modem Control Register - controls modem signals
 * - LSR (0x5): Line Status Register - contains transmission and reception status flags
 * - MSR (0x6): Modem Status Register - contains modem signal states
 * - SCR (0x7): Scratch Register - general purpose register for driver use
 * - DLL (0x0): Divisor Latch Low - accessible when DLAB bit is set in LCR
 * - DLH (0x1): Divisor Latch High - accessible when DLAB bit is set in LCR
 * 
 * Status Flags:
 * - DATA_READY (0x01): Indicates data is available in the receiver buffer
 * - THR_EMPTY (0x20): Indicates transmitter holding register is empty
 * 
 * Frame Control:
 * - FLAG (0x7E): Frame delimiter/synchronization marker
 * - ESCAPE (0x7D): Escape character for special byte sequences
 * - ESCAPE_XOR (0x20): XOR mask applied to escaped bytes
 * 
 * @note Legacy aliases are provided for backward compatibility with older code
 * @note The DLL and DLH registers share addresses with other registers and are
 *       only accessible when the DLAB bit in the LCR is set
 */
 
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

#define SERIAL_DRIVER_CHUNK_SIZE 255u

#endif  // TYPES_H
