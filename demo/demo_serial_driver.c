#include "SerialDriver.h"

int main(void) {
  uint8_t* uart_base = (uint8_t*)0x3F8;
  if (!serial_driver_register_port(SERIAL_PORT_0, uart_base)) {
    return 1;
  }

  serial_descriptor_t descriptor = serial_driver_open(SERIAL_PORT_0);
  if (descriptor == SERIAL_DESCRIPTOR_INVALID) {
    return 1;
  }

  serial_driver_set_discrete(descriptor, MODE_ON);
  serial_driver_set_loopback(descriptor, MODE_ON);
  serial_driver_loopback_enabled(descriptor);
  serial_driver_discrete_enabled(descriptor);
  char write_data[] = "Hello, Serial Port!";
  serial_driver_write(descriptor, (const uint8_t *)write_data, sizeof(write_data));
  uint8_t read_buffer[128];
  serial_driver_read(descriptor, read_buffer, sizeof(read_buffer));
  serial_driver_close(descriptor);
  return 0;
}
