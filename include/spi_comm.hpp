#ifndef SPI_COMM_HPP
#define SPI_COMM_HPP

#include <pico/stdlib.h>
#include <hardware/spi.h>

#include "packet.hpp"

class SPIComm
{
private:
  static constexpr uint32_t SPI_BAUD = 1000000;
  static constexpr uint8_t PIN_SCK = 10;
  static constexpr uint8_t PIN_MISO = 11;
  static constexpr uint8_t PIN_MOSI = 12;
  static constexpr uint8_t PIN_CS = 13;

public:
  static void init_slave()
  {
    spi_init(spi1, SPI_BAUD);
    spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi1, true);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  }

  static bool send_packet(const Packet &packet)
  {
    uint16_t tx_data = packet.to_spi_word();
    uint16_t rx_dummy = 0;

    int result = spi_write16_read16_blocking(spi1, &tx_data, &rx_dummy, 1);

    if (result == 1)
    {
      printf("Packet sent: 0x%04X\n", tx_data);
      return true;
    }
    return false;
  }

  static void print_packet(const Packet &packet)
  {
    printf("Packet - Type: 0x%02X, Action: 0x%02X, Value: 0x%02X, Checksum: 0x%02X, Valid: %s\n",
           static_cast<uint8_t>(packet.type),
           packet.action,
           packet.value,
           packet.checksum,
           packet.is_valid() ? "true" : "false");
  }
};

#endif

// EOF
