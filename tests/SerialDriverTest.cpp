#include <gtest/gtest.h>

extern "C" {
#include "SerialDriver.h"
#include "common_types.h"
#include "circular_buffer.h"
}

namespace {

class SerialDriverFixture : public ::testing::Test {
 protected:
  SerialDriver driver{};

  void SetUp() override {
    serial_driver_init(&driver);
    volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver.UARTbase);
    port[SERIAL_PORT_OFFSET_LSR] = SERIAL_PORT_STATUS_THR_EMPTY;
  }

  void TearDown() override {
    if (driver.UARTbase != nullptr) {
      serial_driver_close(&driver);
    }
  }
};

uint32_t ExpectedEncodedLength(const uint8_t* data, size_t length) {
  uint32_t extra = 0;
  for (size_t i = 0; i < length; ++i) {
    if (data[i] == FLAG || data[i] == ESCAPE) {
      ++extra;
    }
  }
  return static_cast<uint32_t>(length + extra);
}

}  // namespace

TEST_F(SerialDriverFixture, InitSetsRegistersAndBuffers) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver.UARTbase);

  EXPECT_NE(driver.UARTbase, nullptr);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_IER], 0x00);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_LCR], 0x03);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_DLL], 0x01);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_DLH], 0x00);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_FCR], 0xC7);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_MCR], 0x03);
  EXPECT_EQ(cb_size(&driver.tx_cb), 0u);
  EXPECT_EQ(cb_size(&driver.rx_cb), 0u);
}

TEST_F(SerialDriverFixture, SetDiscreteAndLoopbackUpdateState) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver.UARTbase);
  volatile uint8_t* mcr = port + SERIAL_PORT_OFFSET_MCR;

  serial_driver_set_discrete(&driver, MODE_OFF);
  EXPECT_EQ((*mcr & (1u << 1)), 0u);
  EXPECT_FALSE(serial_driver_discrete_enabled(&driver));

  serial_driver_set_discrete(&driver, MODE_ON);
  EXPECT_NE((*mcr & (1u << 1)), 0u);
  EXPECT_TRUE(serial_driver_discrete_enabled(&driver));

  serial_driver_set_loopback(&driver, MODE_ON);
  EXPECT_NE((*mcr & (1u << 4)), 0u);
  EXPECT_TRUE(serial_driver_loopback_enabled(&driver));

  serial_driver_set_loopback(&driver, MODE_OFF);
  EXPECT_EQ((*mcr & (1u << 4)), 0u);
  EXPECT_FALSE(serial_driver_loopback_enabled(&driver));
}

TEST_F(SerialDriverFixture, WriteEncodesAndDrainsToThr) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver.UARTbase);
  uint8_t input[] = {0x01, FLAG, 0x02, ESCAPE};

  uint32_t written = serial_driver_write(&driver, input, sizeof(input));
  uint32_t expected_length = ExpectedEncodedLength(input, sizeof(input));

  EXPECT_EQ(written, expected_length);
  EXPECT_TRUE(cb_is_empty(&driver.tx_cb));
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_THR], static_cast<uint8_t>(ESCAPE ^ ESCAPE_XOR));
}

TEST_F(SerialDriverFixture, ReadUsesRxCacheWhenAvailable) {
  uint8_t cached[] = {0x10, 0x11, 0x12};
  for (uint8_t value : cached) {
    EXPECT_TRUE(cb_push(&driver.rx_cb, value));
  }

  uint8_t out[2] = {};
  uint32_t read = serial_driver_read(&driver, out, sizeof(out));

  EXPECT_EQ(read, sizeof(out));
  EXPECT_EQ(out[0], cached[0]);
  EXPECT_EQ(out[1], cached[1]);
  EXPECT_EQ(cb_size(&driver.rx_cb), 1u);
  EXPECT_TRUE(serial_driver_data_available(&driver));
}

TEST_F(SerialDriverFixture, CloseResetsStateAndFreesMemory) {
  serial_driver_set_discrete(&driver, MODE_ON);
  serial_driver_set_loopback(&driver, MODE_ON);

  serial_driver_close(&driver);

  EXPECT_EQ(driver.UARTbase, nullptr);
  EXPECT_FALSE(serial_driver_discrete_enabled(&driver));
  EXPECT_FALSE(serial_driver_loopback_enabled(&driver));
  EXPECT_EQ(cb_size(&driver.tx_cb), 0u);
  EXPECT_EQ(cb_size(&driver.rx_cb), 0u);
}
