#ifndef SPI_COMM_HPP
#define SPI_COMM_HPP

#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/sync.h>
#include <cstdint>
#include <stdio.h>

extern volatile uint16_t g_rx_word;

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

  static volatile uint16_t tx_data;
  static volatile bool data_ready;
  static volatile bool transmission_in_progress;
  static volatile absolute_time_t last_transmission_time;
  static size_t logan_queue_index;
  static bool logan_queue_active;

  // Dummy data
  static constexpr size_t LOGAN_PACKET_SIZE = 127;
  static inline uint16_t dummy_samples_packet[LOGAN_PACKET_SIZE] = {
    // header
    0x0101,
    // payload: dummy samples 125 16-bit words
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656,    // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, 0x3434, 0x5656, // 10 samples
    0x3434, 0x5656, 0x3434, 0x5656, 0x3434,                                                       // 5  samples
    // checksum
    0x0000
  };

  static inline uint8_t compute_event_checksum(uint8_t type, uint8_t action, uint8_t value)
  {
    return type ^ action ^ value;
  }

  static inline uint16_t compute_logan_checksum(const uint16_t* packet, size_t packet_size)
  {
    if (packet == nullptr || packet_size < 3)
    {
      return 0;
    }

    uint16_t checksum = 0;
    for (size_t i = 1; i + 1 < packet_size; ++i)
    {
      checksum ^= packet[i];
    }
    return checksum;
  }

  static inline uint16_t compute_logan_checksum(const uint16_t (&packet)[LOGAN_PACKET_SIZE])
  {
    uint16_t checksum = 0;
    for (size_t i = 1; i < LOGAN_PACKET_SIZE - 1; ++i)
      checksum ^= packet[i];

    return checksum;
  }

  static inline uint16_t create_event_packet(uint8_t type, uint8_t action, uint8_t value, uint8_t checksum)
  {
    return ((type & 0x0F) << 12) |
           ((action & 0x0F) << 8) |
           ((value & 0x0F) << 4) |
           (checksum & 0x0F);
  }

  static void spi_slave_irq_handler()
  {
    uint32_t status = save_and_disable_interrupts();

    // Clear all pending interrupts
    spi_get_hw(spi1)->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;

    if (spi_is_readable(spi1))
    {
      volatile uint16_t rx_word_local = spi_get_hw(spi1)->dr;
      g_rx_word = rx_word_local;

      if (data_ready && !transmission_in_progress)
      {
        spi_get_hw(spi1)->dr = tx_data;
        data_ready = false;
        transmission_in_progress = true;
        last_transmission_time = get_absolute_time();
      }
      else
      {
        spi_get_hw(spi1)->dr = 0x0000;
      }
    }

    restore_interrupts(status);
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

    // Enable receive interrupt only
    spi_get_hw(spi1)->imsc = SPI_SSPIMSC_RXIM_BITS;
    irq_set_exclusive_handler(SPI1_IRQ, spi_slave_irq_handler);
    irq_set_enabled(SPI1_IRQ, true);

    tx_data = 0;
    data_ready = false;
    transmission_in_progress = false;
    last_transmission_time = get_absolute_time();
  }

  static bool queue_packet(uint16_t packet)
  {
    uint32_t status = save_and_disable_interrupts();

    if (!data_ready && !transmission_in_progress)
    {
      tx_data = packet;
      data_ready = true;
      restore_interrupts(status);
      return true;
    }

    restore_interrupts(status);
    return false;
  }

  static void update_transmission_status()
  {
    if (transmission_in_progress)
    {
      absolute_time_t now = get_absolute_time();
      if (absolute_time_diff_us(last_transmission_time, now) > 10000)
      {
        uint32_t status = save_and_disable_interrupts();
        transmission_in_progress = false;
        restore_interrupts(status);
      }
    }
  }

  static inline uint16_t create_macro_key_event(uint8_t key_num, bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_event_checksum(EVENT_MACRO_KEY, key_num, value);

    return create_event_packet(EVENT_MACRO_KEY, key_num, value, checksum);
  }

  static inline uint16_t create_encoder_rotate_event(bool clockwise, uint8_t steps = 1)
  {
    uint8_t action = clockwise ? ENCODER_CW : ENCODER_CCW;
    uint8_t value = steps & 0x0F;
    uint8_t checksum = compute_event_checksum(EVENT_ENCODER_ROTATE, action, value);

    return create_event_packet(EVENT_ENCODER_ROTATE, action, value, checksum);
  }

  static inline uint16_t create_encoder_switch_event(bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_event_checksum(EVENT_ENCODER_SWITCH, ENCODER_BTN, value);

    return create_event_packet(EVENT_ENCODER_SWITCH, ENCODER_BTN, value, checksum);
  }

  static inline uint16_t create_samples_packet(uint16_t samples)
  {
    samples &= 0x0FFF;

    uint8_t checksum = ((samples >> 8) & 0xFF) ^ (samples & 0xFF);
    checksum &= 0x0F;

    return (samples << 4) | checksum;
  }

  static bool send_packet(uint16_t tx_buffer)
  {
    uint16_t rx_buffer = 0;
    int result = spi_write16_read16_blocking(spi1, &tx_buffer, &rx_buffer, 1);

    if (result == 1)
    {
      uint8_t tx_high_byte = (tx_buffer >> 8) & 0xFF;
      uint8_t tx_low_byte = tx_buffer & 0xFF;
      uint8_t rx_high_byte = (rx_buffer >> 8) & 0xFF;
      uint8_t rx_low_byte = rx_buffer & 0xFF;
      printf("Packet Sent: 0x%04X (bytes: 0x%02X 0x%02X) | Received: 0x%04X (bytes: 0x%02X 0x%02X)\n",
             tx_buffer, tx_high_byte, tx_low_byte, rx_buffer, rx_high_byte, rx_low_byte);
      return true;
    }
    return false;
  }

  static bool is_transmission_ready()
  {
    return !data_ready && !transmission_in_progress;
  }

  static uint8_t get_queue_status()
  {
    if (transmission_in_progress) return 2;
    if (data_ready) return 1;
    return 0;
  }

  static inline void start_dummy_samples_transfer()
  {
    if (logan_queue_active)
      return;

    uint16_t checksum = compute_logan_checksum(dummy_samples_packet);
    dummy_samples_packet[LOGAN_PACKET_SIZE - 1] = checksum;
    logan_queue_index = 0;
    logan_queue_active = true;
  }

  static inline bool has_dummy_samples_pending()
  {
    return logan_queue_active && (logan_queue_index < LOGAN_PACKET_SIZE);
  }

  static inline bool try_get_next_dummy_word(uint16_t &out)
  {
    if (!has_dummy_samples_pending())
      return false;

    out = dummy_samples_packet[logan_queue_index++];
    if (logan_queue_index >= LOGAN_PACKET_SIZE)
      logan_queue_active = false;
    return true;
  }
};

volatile uint16_t SPIComm::tx_data = 0;
volatile bool SPIComm::data_ready = false;
volatile bool SPIComm::transmission_in_progress = false;
volatile absolute_time_t SPIComm::last_transmission_time = 0;
size_t SPIComm::logan_queue_index = 0;
bool SPIComm::logan_queue_active = false;

#endif

// EOF
