#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "circular_buffer.h"
#include "common_types.h"

#define SERIAL_DRIVER_TX_BUFFER_SIZE 256
#define SERIAL_DRIVER_RX_BUFFER_SIZE 256
#define SERIAL_DRIVER_PORT_COUNT 8

typedef enum {
  SERIAL_PORT_0 = 0,
  SERIAL_PORT_1,
  SERIAL_PORT_2,
  SERIAL_PORT_3,
  SERIAL_PORT_4,
  SERIAL_PORT_5,
  SERIAL_PORT_6,
  SERIAL_PORT_7
} serial_port_t;

typedef int32_t serial_descriptor_t;

#define SERIAL_DESCRIPTOR_INVALID ((serial_descriptor_t)-1)

/**
 * @struct SerialDriver
 * @brief Structure for managing UART serial communication with buffering capabilities
 *
 * @details Encapsulates all necessary components for serial driver operations including
 * UART hardware access, operational modes, and transmit/receive circular buffers with
 * pre-allocated storage.
 *
 * @member UARTbase Pointer to the base address of the UART hardware peripheral
 * @member loopback_mode Flag indicating whether loopback mode is enabled for testing
 * @member discrete_mode Flag indicating whether discrete mode is enabled
 * @member tx_cb Circular buffer structure for managing transmit queue
 * @member rx_cb Circular buffer structure for managing receive queue
 * @member tx_storage Pre-allocated storage array for transmit circular buffer data
 * @member rx_storage Pre-allocated storage array for receive circular buffer data
 */
typedef struct SerialDriver {
  uint8_t* UARTbase;
  status_t loopback_mode;
  status_t discrete_mode;
  circular_buffer_t tx_cb;
  circular_buffer_t rx_cb;
  uint32_t tx_storage[SERIAL_DRIVER_TX_BUFFER_SIZE];
  uint32_t rx_storage[SERIAL_DRIVER_RX_BUFFER_SIZE];
} SerialDriver;


bool serial_driver_register_port(serial_port_t port, uint8_t* base);
serial_descriptor_t serial_driver_open(serial_port_t port);
SerialDriver* serial_driver_get(serial_descriptor_t descriptor);
/**
 * @brief Writes data to the specified serial driver.
 *
 * This function transmits a sequence of bytes from the provided buffer through the given SerialDriver instance.
 *
 * @param descriptor Serial descriptor for the driver instance to which data will be written.
 * @param buffer Pointer to the buffer containing the data to be written.
 * @param length Number of bytes to write from the buffer.
 * @return The number of bytes successfully written, or 0 if an error occurred.
 */
uint32_t serial_driver_write(serial_descriptor_t descriptor, const uint8_t* buffer, size_t length);
/**
 * @brief Reads data from the specified serial driver into a buffer.
 *
 * This function attempts to read up to 'length' bytes from the serial driver
 * and stores the data in the provided 'buffer'. The actual number of bytes
 * read may be less than 'length' if fewer bytes are available.
 *
 * @param descriptor Serial descriptor for the driver instance to read from.
 * @param buffer Pointer to the buffer where the read data will be stored.
 * @param length Maximum number of bytes to read.
 * @return The number of bytes actually read from the serial driver.
 */
uint32_t serial_driver_read(serial_descriptor_t descriptor, uint8_t* buffer, uint32_t length);
/**
 * @brief Closes the specified serial driver and releases any associated resources.
 *
 * This function terminates the connection managed by the given SerialDriver instance,
 * ensuring that all resources such as file descriptors or handles are properly released.
 * After calling this function, the SerialDriver instance should not be used unless re-initialized.
 *
 * @param descriptor Serial descriptor for the driver instance to be closed.
 */
void serial_driver_close(serial_descriptor_t descriptor);
/**
 * @brief Sets the discrete mode of the specified SerialDriver instance.
 *
 * This function configures the given SerialDriver to operate in the specified discrete mode.
 *
 * @param descriptor Serial descriptor for the driver instance to configure.
 * @param mode The discrete mode to set for the driver.
 */
void serial_driver_set_discrete(serial_descriptor_t descriptor, status_t mode);
/**
 * @brief Sets the loopback mode for the specified SerialDriver instance.
 *
 * This function enables or disables the loopback mode on the given serial driver.
 * When loopback mode is enabled, data sent to the driver is immediately received back,
 * which is useful for testing and diagnostics.
 *
 * @param descriptor Serial descriptor for the driver instance to configure.
 * @param mode   The desired loopback mode (typically enable or disable).
 */
void serial_driver_set_loopback(serial_descriptor_t descriptor, status_t mode);
/**
 * @brief Checks if loopback mode is enabled for the specified serial driver.
 *
 * Loopback mode is typically used for testing purposes, where transmitted data is
 * internally routed back to the receiver without leaving the device.
 *
 * @param descriptor Serial descriptor for the driver instance to check.
 * @return true if loopback mode is enabled, false otherwise.
 */
bool serial_driver_loopback_enabled(serial_descriptor_t descriptor);
/**
 * @brief Checks if the discrete mode is enabled for the specified serial driver.
 *
 * This function determines whether the given SerialDriver instance has
 * discrete mode enabled. Discrete mode may refer to a specific operational
 * mode of the serial driver, such as handling data in discrete packets
 * rather than a continuous stream.
 *
 * @param descriptor Serial descriptor for the driver instance to query.
 * @return true if discrete mode is enabled, false otherwise.
 */
bool serial_driver_discrete_enabled(serial_descriptor_t descriptor);
/**
 * @brief Checks if there is data available to read from the serial driver's receive buffer.
 *
 * @param descriptor Serial descriptor for the driver instance to query.
 * @return true if data is available to read; false otherwise.
 */
bool serial_driver_data_available(serial_descriptor_t descriptor);
/**
 * @brief Checks if the serial driver's transmitter is empty.
 *
 * @param descriptor Serial descriptor for the driver instance to query.
 * @return true if the transmitter is empty, false otherwise.
 */
bool serial_driver_transmitter_empty(serial_descriptor_t descriptor);

#endif
