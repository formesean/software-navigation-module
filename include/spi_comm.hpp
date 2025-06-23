#ifndef SPI_COMM_HPP
#define SPI_COMM_HPP

#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <cstdint>

class SPIComm
{
private:
  static constexpr uint32_t SPI_BAUD = 20000000;
  static constexpr uint8_t PIN_SCK = 10;
  static constexpr uint8_t PIN_MISO = 11;
  static constexpr uint8_t PIN_MOSI = 12;
  static constexpr uint8_t PIN_CS = 13;

  static constexpr uint8_t EVENT_MACRO_KEY = 0x01;
  static constexpr uint8_t EVENT_ENCODER_ROTATE = 0x02;
  static constexpr uint8_t EVENT_ENCODER_SWITCH = 0x03;

  static constexpr uint8_t ENCODER_CW = 0x01;
  static constexpr uint8_t ENCODER_CCW = 0x02;
  static constexpr uint8_t ENCODER_BTN = 0x01;

  static constexpr uint8_t STATE_PRESSED = 0x01;
  static constexpr uint8_t STATE_RELEASED = 0x00;

  static inline uint8_t compute_checksum(uint8_t type, uint8_t action, uint8_t value)
  {
    return type ^ action ^ value;
  }

  static inline uint16_t pack_to_16bit(uint8_t type, uint8_t action, uint8_t value, uint8_t checksum)
  {
    return ((type & 0x0F) << 12) |
           ((action & 0x0F) << 8) |
           ((value & 0x0F) << 4) |
           (checksum & 0x0F);
  }

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

  static inline uint16_t create_macro_key_event(uint8_t key_num, bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_checksum(EVENT_MACRO_KEY, key_num, value);

    return pack_to_16bit(EVENT_MACRO_KEY, key_num, value, checksum);
  }

  static inline uint16_t create_encoder_rotate_event(bool clockwise, uint8_t steps = 1)
  {
    uint8_t action = clockwise ? ENCODER_CW : ENCODER_CCW;
    uint8_t value = steps & 0x0F;
    uint8_t checksum = compute_checksum(EVENT_ENCODER_ROTATE, action, value);

    return pack_to_16bit(EVENT_ENCODER_ROTATE, action, value, checksum);
  }

  static inline uint16_t create_encoder_switch_event(bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_checksum(EVENT_ENCODER_SWITCH, ENCODER_BTN, value);

    return pack_to_16bit(EVENT_ENCODER_SWITCH, ENCODER_BTN, value, checksum);
  }

  static bool send_packet(uint16_t tx_buffer)
  {
    uint16_t rx_buffer = 0;
    int result = spi_write16_read16_blocking(spi1, &tx_buffer, &rx_buffer, 1);

    if (result == 1)
    {
      printf("Packet sent: 0x%04X\n", tx_buffer);
      return true;
    }
    return false;
  }
};

#endif

// EOF
