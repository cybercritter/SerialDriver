#include <array>
#include <gtest/gtest.h>

extern "C" {
#include "SerialDriver.h"
#include "common_types.h"
#include "circular_buffer.h"
}

namespace {

constexpr size_t kUartMemorySize = 256;
using UartMemory = std::array<uint8_t, kUartMemorySize>;
static std::array<UartMemory, SERIAL_DRIVER_PORT_COUNT> g_uart_ports{};

class SerialDriverFixture : public ::testing::TestWithParam<int> {
 protected:
  serial_port_t port_id{};
  SerialDriver* driver = nullptr;

  void SetUp() override {
    port_id = static_cast<serial_port_t>(GetParam());
    ASSERT_TRUE(serial_driver_register_port(port_id, g_uart_ports[port_id].data()));
    ASSERT_TRUE(serial_driver_init_port(port_id));
    driver = serial_driver_get(port_id);
    ASSERT_NE(driver, nullptr);
    volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver->UARTbase);
    port[SERIAL_PORT_OFFSET_LSR] = SERIAL_PORT_STATUS_THR_EMPTY;
  }

  void TearDown() override {
    if (driver != nullptr && driver->UARTbase != nullptr) {
      serial_driver_close(driver);
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

TEST_P(SerialDriverFixture, InitSetsRegistersAndBuffers) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver->UARTbase);

  EXPECT_NE(driver->UARTbase, nullptr);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_IER], 0x00);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_LCR], 0x03);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_DLL], 0x01);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_DLH], 0x00);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_FCR], 0xC7);
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_MCR], 0x03);
  EXPECT_EQ(cb_size(&driver->tx_cb), 0u);
  EXPECT_EQ(cb_size(&driver->rx_cb), 0u);
}

TEST_P(SerialDriverFixture, SetDiscreteAndLoopbackUpdateState) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver->UARTbase);
  volatile uint8_t* mcr = port + SERIAL_PORT_OFFSET_MCR;

  serial_driver_set_discrete(driver, MODE_OFF);
  EXPECT_EQ((*mcr & (1u << 1)), 0u);
  EXPECT_FALSE(serial_driver_discrete_enabled(driver));

  serial_driver_set_discrete(driver, MODE_ON);
  EXPECT_NE((*mcr & (1u << 1)), 0u);
  EXPECT_TRUE(serial_driver_discrete_enabled(driver));

  serial_driver_set_loopback(driver, MODE_ON);
  EXPECT_NE((*mcr & (1u << 4)), 0u);
  EXPECT_TRUE(serial_driver_loopback_enabled(driver));

  serial_driver_set_loopback(driver, MODE_OFF);
  EXPECT_EQ((*mcr & (1u << 4)), 0u);
  EXPECT_FALSE(serial_driver_loopback_enabled(driver));
}

TEST_P(SerialDriverFixture, WriteEncodesAndDrainsToThr) {
  volatile uint8_t* port = reinterpret_cast<volatile uint8_t*>(driver->UARTbase);
  uint8_t input[] = {0x01, FLAG, 0x02, ESCAPE};

  uint32_t written = serial_driver_write(driver, input, sizeof(input));
  uint32_t expected_length = ExpectedEncodedLength(input, sizeof(input));

  EXPECT_EQ(written, expected_length);
  EXPECT_TRUE(cb_is_empty(&driver->tx_cb));
  EXPECT_EQ(port[SERIAL_PORT_OFFSET_THR], static_cast<uint8_t>(ESCAPE ^ ESCAPE_XOR));
}

TEST_P(SerialDriverFixture, SendAndReceiveData) {
  uint8_t to_send[] = {0xAA, 0xBB, 0xCC};
  uint32_t written = serial_driver_write(driver, to_send, sizeof(to_send));
  EXPECT_EQ(written, ExpectedEncodedLength(to_send, sizeof(to_send)));

  uint8_t to_receive[] = {0x10, 0x20, 0x30};
  for (uint8_t value : to_receive) {
    EXPECT_TRUE(cb_push(&driver->rx_cb, value));
  }

  uint8_t out[3] = {};
  uint32_t read = serial_driver_read(driver, out, sizeof(out));

  EXPECT_EQ(read, sizeof(out));
  EXPECT_EQ(out[0], to_receive[0]);
  EXPECT_EQ(out[1], to_receive[1]);
  EXPECT_EQ(out[2], to_receive[2]);
  EXPECT_TRUE(cb_is_empty(&driver->rx_cb));
}

TEST_P(SerialDriverFixture, ReadUsesRxCacheWhenAvailable) {
  uint8_t cached[] = {0x10, 0x11, 0x12};
  for (uint8_t value : cached) {
    EXPECT_TRUE(cb_push(&driver->rx_cb, value));
  }

  uint8_t out[2] = {};
  uint32_t read = serial_driver_read(driver, out, sizeof(out));

  EXPECT_EQ(read, sizeof(out));
  EXPECT_EQ(out[0], cached[0]);
  EXPECT_EQ(out[1], cached[1]);
  EXPECT_EQ(cb_size(&driver->rx_cb), 1u);
  // EXPECT_TRUE(serial_driver_data_available(driver));
}

TEST_P(SerialDriverFixture, CloseResetsStateAndFreesMemory) {
  serial_driver_set_discrete(driver, MODE_ON);
  serial_driver_set_loopback(driver, MODE_ON);

  serial_driver_close(driver);

  EXPECT_EQ(driver->UARTbase, nullptr);
  EXPECT_FALSE(serial_driver_discrete_enabled(driver));
  EXPECT_FALSE(serial_driver_loopback_enabled(driver));
  EXPECT_EQ(cb_size(&driver->tx_cb), 0u);
  EXPECT_EQ(cb_size(&driver->rx_cb), 0u);
}

INSTANTIATE_TEST_SUITE_P(
    AllPorts,
    SerialDriverFixture,
    ::testing::Range(0, static_cast<int>(SERIAL_DRIVER_PORT_COUNT)));
