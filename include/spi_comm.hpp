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
  static constexpr uint32_t SPI_BAUD = 30000000;
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

  // TX FIFO based streaming state
  static size_t logan_queue_index;
  static bool logan_queue_active;

  // Unified TX transaction queue (events and bulk transfers)
  enum TxItemType : uint8_t { TX_EVENT = 1, TX_BULK = 2 };
  static constexpr size_t TXQ_CAPACITY = 64;
  static uint8_t txq_type[TXQ_CAPACITY];
  static uint16_t txq_word[TXQ_CAPACITY];
  static uint32_t txq_head;
  static uint32_t txq_tail;
  static uint32_t txq_count;
  static bool logan_queued; // bulk transfer queued but not yet active

  // Dynamic LOGAN dummy streaming state
  static uint16_t logan_header_word;        // word[0]
  static uint16_t logan_checksum_accum;     // XOR over payload
  static size_t logan_payload_words;        // number of payload sample words
  static size_t logan_total_words;          // payload + header + checksum

  static inline uint8_t compute_event_checksum(uint8_t type, uint8_t action, uint8_t value)
  {
    return type ^ action ^ value;
  }

  // Map samples nibble to number of 16-bit payload sample words
  static inline size_t map_samples_nibble(uint8_t samples_nibble)
  {
    switch (samples_nibble & 0x0F)
    {
      case 0xA: return 2000;
      case 0x9: return 1000;
      case 0x8: return 500;
      case 0x7: return 200;
      case 0x6: return 100;
      case 0x5: return 50;
      case 0x4: return 20;
      case 0x3: return 10;
      case 0x2: return 5;
      case 0x1: return 2;
      default:  return 10; // safe default
    }
  }

  // Simple payload generator: repeat pattern 0x3434, 0x5656
  static inline uint16_t generate_sample_word(bool &toggle)
  {
    toggle = !toggle;
    return toggle ? 0x0101 : 0x1010;
  }

  static inline uint16_t create_event_packet(uint8_t type, uint8_t action, uint8_t value, uint8_t checksum)
  {
    return ((type & 0x0F) << 12) |
           ((action & 0x0F) << 8) |
           ((value & 0x0F) << 4) |
           (checksum & 0x0F);
  }

  // Helper to check TX FIFO space
  static inline bool tx_fifo_not_full()
  {
    return (spi_get_hw(spi1)->sr & SPI_SSPSR_TNF_BITS) != 0;
  }

  static inline bool tx_fifo_empty()
  {
    return (spi_get_hw(spi1)->sr & SPI_SSPSR_TFE_BITS) != 0;
  }

  // Fill as many dummy words as possible into TX FIFO
  static inline void preload_dummy_tx_fifo()
  {
    while (tx_fifo_not_full() && has_dummy_samples_pending())
    {
      uint16_t word;
      if (!try_get_next_dummy_word(word))
        break;
      spi_get_hw(spi1)->dr = word;
    }
  }

  // Core implementation: assumes interrupts are disabled by caller
  static inline void service_tx_fifo_locked()
  {
    while (tx_fifo_not_full())
    {
      // Continue active bulk first
      if (logan_queue_active)
      {
        preload_dummy_tx_fifo();
        if (!tx_fifo_not_full()) break;
        if (has_dummy_samples_pending()) continue;
        // Bulk finished
        logan_queue_active = false;
        continue;
      }

      if (txq_count == 0)
      {
        break;
      }

      uint8_t item_type = txq_type[txq_tail];
      if (item_type == TX_EVENT)
      {
        uint16_t word = txq_word[txq_tail];
        txq_tail = (txq_tail + 1) & (TXQ_CAPACITY - 1);
        --txq_count;
        if (!tx_fifo_not_full())
        {
          txq_tail = (txq_tail + TXQ_CAPACITY - 1) & (TXQ_CAPACITY - 1);
          ++txq_count;
          txq_word[txq_tail] = word;
          txq_type[txq_tail] = TX_EVENT;
          break;
        }
        spi_get_hw(spi1)->dr = word;
        continue;
      }
      else // TX_BULK
      {
        txq_tail = (txq_tail + 1) & (TXQ_CAPACITY - 1);
        --txq_count;

        // start bulk
        logan_queue_index = 0;
        logan_queue_active = true;
        logan_queued = false;
        preload_dummy_tx_fifo();
        if (!tx_fifo_not_full()) break;
        continue;
      }
    }
  }

  // Public-safe wrapper: handles interrupt state internally
  static inline void service_tx_fifo()
  {
    uint32_t status = save_and_disable_interrupts();
    service_tx_fifo_locked();
    restore_interrupts(status);
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
      // Service TX FIFO with queued transactions (interrupts already disabled)
      service_tx_fifo_locked();
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
  }

  static bool queue_packet(uint16_t packet)
  {
    uint32_t status = save_and_disable_interrupts();

    // Fast-path: direct write only if absolutely no sequencing pending
    if (!logan_queue_active && txq_count == 0 && tx_fifo_not_full())
    {
      spi_get_hw(spi1)->dr = packet;
      restore_interrupts(status);
      return true;
    }

    // Enqueue as an event transaction
    if (txq_count < TXQ_CAPACITY)
    {
      txq_type[txq_head] = TX_EVENT;
      txq_word[txq_head] = packet;
      txq_head = (txq_head + 1) & (TXQ_CAPACITY - 1);
      ++txq_count;
      restore_interrupts(status);
      // Try to kick the TX engine
      service_tx_fifo();
      return true;
    }

    restore_interrupts(status);
    return false;
  }

  // Opportunistically top-up TX FIFO with dummy samples
  static void update_transmission_status()
  {
    service_tx_fifo();
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

  // Configure dynamic LOGAN stream based on RX nibbles.
  // Sets header word and payload length; checksum is accumulated on the fly.
  static inline void configure_dummy_header_from_rx(uint8_t samples_nibble, uint8_t rate_nibble)
  {
    uint8_t type = 0x06;
    uint8_t action = samples_nibble & 0x0F;
    uint8_t value = rate_nibble & 0x0F;
    uint8_t checksum = compute_event_checksum(type, action, value) & 0x0F;

    logan_header_word = ((type & 0x0F) << 12) |
                        ((action & 0x0F) << 8) |
                        ((value & 0x0F) << 4) |
                        (checksum & 0x0F);

    logan_payload_words = map_samples_nibble(samples_nibble);
    logan_total_words = 1 /*header*/ + logan_payload_words + 1 /*checksum*/;
    logan_checksum_accum = 0;
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
    return tx_fifo_not_full();
  }

  static uint8_t get_queue_status()
  {
    // 2: data in FIFO (transmitting/ready), 1: FIFO full, 0: FIFO has space/empty
    if (!tx_fifo_empty()) return 2;
    if (!tx_fifo_not_full()) return 1;
    return 0;
  }

  static inline void start_dummy_samples_transfer()
  {
    // Avoid duplicate scheduling
    if (logan_queue_active || logan_queued)
      return;

    uint32_t status = save_and_disable_interrupts();
    if (txq_count < TXQ_CAPACITY)
    {
      txq_type[txq_head] = TX_BULK;
      txq_word[txq_head] = 0;
      txq_head = (txq_head + 1) & (TXQ_CAPACITY - 1);
      ++txq_count;
      logan_queued = true;
      restore_interrupts(status);
      service_tx_fifo();
      return;
    }
    restore_interrupts(status);
  }

  static inline bool has_dummy_samples_pending()
  {
    return logan_queue_active && (logan_queue_index < logan_total_words);
  }

  static inline bool try_get_next_dummy_word(uint16_t &out)
  {
    if (!has_dummy_samples_pending())
      return false;

    // index 0: header
    if (logan_queue_index == 0)
    {
      out = logan_header_word;
      logan_queue_index++;
      return true;
    }

    // last index: checksum word
    if (logan_queue_index == (logan_total_words - 1))
    {
      out = logan_checksum_accum;
      logan_queue_index++;
      logan_queue_active = false;
      return true;
    }

    // payload sample
    static bool toggle = false;
    uint16_t sample = generate_sample_word(toggle);
    logan_checksum_accum ^= sample;
    out = sample;
    logan_queue_index++;
    return true;
  }
};

size_t SPIComm::logan_queue_index = 0;
bool SPIComm::logan_queue_active = false;
uint8_t SPIComm::txq_type[SPIComm::TXQ_CAPACITY] = {};
uint16_t SPIComm::txq_word[SPIComm::TXQ_CAPACITY] = {};
uint32_t SPIComm::txq_head = 0;
uint32_t SPIComm::txq_tail = 0;
uint32_t SPIComm::txq_count = 0;
bool SPIComm::logan_queued = false;

// Dynamic LOGAN streaming state definitions
uint16_t SPIComm::logan_header_word = 0;
uint16_t SPIComm::logan_checksum_accum = 0;
size_t SPIComm::logan_payload_words = 0;
size_t SPIComm::logan_total_words = 0;

#endif

// EOF
