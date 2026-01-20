#include "SerialDriver.h"

int main(void) {
  struct SerialDriver driver;
  serial_driver_init(&driver);
  serial_driver_set_discrete(&driver, MODE_ON);
  serial_driver_set_loopback(&driver, MODE_ON);
  serial_driver_loopback_enabled(&driver);
  serial_driver_discrete_enabled(&driver);
  char write_data[] = "Hello, Serial Port!";
  serial_driver_write(&driver, (const uint8_t *)write_data);
  uint8_t read_buffer[128];
  serial_driver_read(&driver, read_buffer, sizeof(read_buffer));
  serial_driver_close(&driver);
  return 0;
}
