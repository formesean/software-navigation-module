#ifndef SPI_COMM_HPP
#define SPI_COMM_HPP

#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/sync.h>
#include <cstdint>
#include <stdio.h>

// SPI slave communication utilities for event packets and LOGAN data streaming (RP2040 SPI1)

extern volatile uint16_t g_rx_word;
extern volatile bool g_stop_requested;
extern volatile bool g_start_requested;

// SPI slave communication: queues high-priority events and streams LOGAN sample payloads.
class SPIComm
{
private:
  static constexpr uint32_t SPI_BAUD = 10000000;
  static constexpr uint8_t PIN_SCK = 10;
  static constexpr uint8_t PIN_MISO = 11;
  static constexpr uint8_t PIN_MOSI = 12;
  static constexpr uint8_t PIN_CS = 13;

  static constexpr uint16_t CMD_START = 0xA11A;
  static constexpr uint16_t CMD_STOP  = 0xA10B;

  static constexpr uint8_t EVENT_MACRO_KEY = 0x01;
  static constexpr uint8_t EVENT_ENCODER_ROTATE = 0x02;
  static constexpr uint8_t EVENT_ENCODER_SWITCH = 0x03;
  static constexpr uint8_t EVENT_SNM_ANNOUNCE = 0x0A;

  static constexpr uint8_t ENCODER_CW = 0x01;
  static constexpr uint8_t ENCODER_CCW = 0x02;
  static constexpr uint8_t ENCODER_BTN = 0x01;

  static constexpr uint8_t STATE_PRESSED = 0x01;
  static constexpr uint8_t STATE_RELEASED = 0x00;

  static constexpr uint8_t SNM_ANNOUNCE_ACTION = 0x01;
  static constexpr uint8_t SNM_ATTACHED_VALUE = 0x01;
  static constexpr uint8_t SNM_STOPPING_VALUE = 0x00;

  // TX FIFO based streaming state
  static size_t logan_queue_index;
  static bool logan_queue_active;

  // Unified TX transaction queue (events and bulk transfers)
  enum TxItemType : uint8_t { TX_EVENT = 1, TX_BULK = 2 };
  static constexpr size_t TXQ_CAPACITY = 128;
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

  // LOGAN payload storage (filled externally before transmission)
  static constexpr size_t LOGAN_MAX_SAMPLES = 2000;
  static uint16_t logan_payload_storage[LOGAN_MAX_SAMPLES];

  // 0x101

  static inline uint8_t compute_event_checksum(uint8_t type, uint8_t action, uint8_t value)
  {
    return type ^ action ^ value;
  }

  static inline bool is_control_word(uint16_t w)
  {
    return (w == CMD_START) || (w == CMD_STOP);
  }

  // Map samples nibble to number of 1-bit samples
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
      default:  return 500;
    }
  }

  static inline size_t map_sampling_rate_nibble(uint8_t sampling_rate_nibble)
  {
    switch (sampling_rate_nibble & 0x0F)
    {
      case 0xA: return 1000;
      case 0x9: return 500;
      case 0x8: return 200;
      case 0x7: return 100;
      case 0x6: return 50;
      case 0x5: return 20;
      case 0x4: return 10;
      case 0x3: return 5;
      case 0x2: return 2;
      case 0x1: return 1;
      default:  return 50;
    }
  }

  // Simple payload generator (unused helper): alternating nibble pattern
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
      // Continue active bulk, but interleave high-priority events between payload words
      if (logan_queue_active)
      {
        // If an event is pending, emit it first
        if (txq_count > 0 && txq_type[txq_tail] == TX_EVENT)
        {
          uint16_t word = txq_word[txq_tail];
          txq_tail = (txq_tail + 1) & (TXQ_CAPACITY - 1);
          --txq_count;
          if (!tx_fifo_not_full())
          {
            // push back if no space
            txq_tail = (txq_tail + TXQ_CAPACITY - 1) & (TXQ_CAPACITY - 1);
            ++txq_count;
            txq_word[txq_tail] = word;
            txq_type[txq_tail] = TX_EVENT;
            break;
          }
          spi_get_hw(spi1)->dr = word;
          continue;
        }

        // Otherwise, emit a single payload word
        uint16_t payload_word = 0;
        if (try_get_next_dummy_word(payload_word))
        {
          spi_get_hw(spi1)->dr = payload_word;
          continue;
        }
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

  // SPI1 slave ISR: drains RX FIFO, latches control flags, and services TX FIFO
  static void spi_slave_irq_handler()
  {
    // Keep ISR preemptible: avoid globally disabling interrupts

    // Clear pending RX interrupts
    spi_get_hw(spi1)->icr = SPI_SSPICR_RTIC_BITS | SPI_SSPICR_RORIC_BITS;

    bool stop_seen = false;
    bool start_seen = false;
    uint16_t last_non_control = 0;
    // Allow masters that clock 8-bit frames (CS held across two bytes)
    static uint8_t partial_high_byte = 0;
    static bool partial_high_valid = false;

    // Drain RX FIFO
    while (spi_is_readable(spi1))
    {
      uint16_t w = spi_get_hw(spi1)->dr;

      // If master sends 8-bit frames, assemble two bytes into one word
      if (w <= 0x00FF)
      {
        if (!partial_high_valid)
        {
          partial_high_byte = static_cast<uint8_t>(w & 0xFF);
          partial_high_valid = true;
          continue;
        }
        else
        {
          w = static_cast<uint16_t>((static_cast<uint16_t>(partial_high_byte) << 8) |
                                    static_cast<uint16_t>(w & 0xFF));
          partial_high_valid = false;
        }
      }
      else
      {
        // Any 16-bit frame clears partial state
        partial_high_valid = false;
      }

      if (is_control_word(w))
      {
        if (w == CMD_STOP) { stop_seen = true; }
        else               { start_seen = true; }
        continue;
      }
      // Only update last_non_control for non-zero words to avoid overwriting with zeros
      if (w != 0) { last_non_control = w; }
    }
    // If FIFO drained with a dangling high byte, drop it to avoid false triggers
    partial_high_valid = false;

    // Latch control commands as flags; do not publish them into g_rx_word
    bool control_seen = stop_seen || start_seen;
    if (stop_seen) { g_stop_requested = true; g_rx_word = 0; }
    if (start_seen) { g_start_requested = true; g_rx_word = 0; }
    // Capture data only when no control command was present in the batch
    if (!control_seen && last_non_control != 0) g_rx_word = last_non_control;

    // Service TX FIFO without disabling interrupts to keep GPIO IRQs responsive
    service_tx_fifo_locked();
  }

public:
  struct ControlFlags
  {
    bool start;
    bool stop;
  };

  static constexpr uint16_t start_word() { return CMD_START; }
  static constexpr uint16_t stop_word()  { return CMD_STOP;  }

  // True if a LOGAN bulk transfer is queued or currently active
  static inline bool is_logan_transfer_busy()
  {
    return logan_queue_active || logan_queued;
  }

  // Atomically capture and clear any latched control commands (START/STOP).
  // Also consumes a pending g_rx_word if it equals a control word, ensuring
  // control commands are never misinterpreted as data.
  static inline ControlFlags consume_control_flags()
  {
    ControlFlags flags{false, false};

    uint32_t status = save_and_disable_interrupts();

    if (g_stop_requested)
    {
      flags.stop = true;
      g_stop_requested = false;
    }
    if (g_start_requested)
    {
      flags.start = true;
      g_start_requested = false;
    }

    uint16_t rx_snapshot = g_rx_word;
    if (is_control_word(rx_snapshot))
    {
      flags.start = flags.start || (rx_snapshot == CMD_START);
      flags.stop  = flags.stop  || (rx_snapshot == CMD_STOP);
      g_rx_word = 0;
    }
    else if (flags.start || flags.stop)
    {
      // Drop any stale non-control word when a control command is present.
      g_rx_word = 0;
    }

    restore_interrupts(status);
    return flags;
  }

  // Raise control requests from non-ISR contexts with interrupt safety.
  static inline void signal_stop_request(bool clear_rx_word = true)
  {
    uint32_t status = save_and_disable_interrupts();
    g_stop_requested = true;
    if (clear_rx_word) g_rx_word = 0;
    restore_interrupts(status);
  }

  static inline void signal_start_request(bool clear_rx_word = true)
  {
    uint32_t status = save_and_disable_interrupts();
    g_start_requested = true;
    if (clear_rx_word) g_rx_word = 0;
    restore_interrupts(status);
  }

  // Initialize SPI1 as a 16-bit slave and enable RX interrupt
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
    // Lower SPI1 IRQ priority so GPIO events can preempt during heavy transfers
    irq_set_priority(SPI1_IRQ, 0xA0);
  }

  // Enqueue a single 16-bit event packet; returns false if the queue is full
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
      // Try to kick the TX engine without disabling interrupts
      service_tx_fifo_locked();
      return true;
    }

    restore_interrupts(status);
    return false;
  }

  // Opportunistically service TX FIFO (events and bulk)
  static void update_transmission_status()
  {
    service_tx_fifo();
  }

  // Build an event packet for a macro key press/release
  static inline uint16_t create_macro_key_event(uint8_t key_num, bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_event_checksum(EVENT_MACRO_KEY, key_num, value);

    return create_event_packet(EVENT_MACRO_KEY, key_num, value, checksum);
  }

  // Build an event packet for encoder rotation (CW/CCW and step count)
  static inline uint16_t create_encoder_rotate_event(bool clockwise, uint8_t steps = 1)
  {
    uint8_t action = clockwise ? ENCODER_CW : ENCODER_CCW;
    uint8_t value = steps & 0x0F;
    uint8_t checksum = compute_event_checksum(EVENT_ENCODER_ROTATE, action, value);

    return create_event_packet(EVENT_ENCODER_ROTATE, action, value, checksum);
  }

  // Build an event packet for encoder button press/release
  static inline uint16_t create_encoder_switch_event(bool pressed)
  {
    uint8_t value = pressed ? STATE_PRESSED : STATE_RELEASED;
    uint8_t checksum = compute_event_checksum(EVENT_ENCODER_SWITCH, ENCODER_BTN, value);

    return create_event_packet(EVENT_ENCODER_SWITCH, ENCODER_BTN, value, checksum);
  }

  // Build an event packet for SNM announce/ack (attached=true/false)
  static inline uint16_t create_snm_announce_event(bool attached)
  {
    uint8_t value = attached ? SNM_ATTACHED_VALUE : SNM_STOPPING_VALUE;
    uint8_t checksum = compute_event_checksum(EVENT_SNM_ANNOUNCE, SNM_ANNOUNCE_ACTION, value);

    return create_event_packet(EVENT_SNM_ANNOUNCE, SNM_ANNOUNCE_ACTION, value, checksum);
  }

  // Build a 12-bit samples word with XOR checksum in low nibble
  static inline uint16_t create_samples_packet(uint16_t samples)
  {
    samples &= 0x0FFF;

    uint8_t checksum = ((samples >> 8) & 0xFF) ^ (samples & 0xFF);
    checksum &= 0x0F;

    return (samples << 4) | checksum;
  }

  // Configure dynamic LOGAN stream from RX nibbles (sets header and payload length)
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

    size_t samples = map_samples_nibble(samples_nibble);
    logan_payload_words = (samples + 3) / 4; // convert samples to 16-bit payload words
    logan_total_words = 1 /*header*/ + logan_payload_words + 1 /*checksum*/;
    logan_checksum_accum = 0;
  }

  // Synchronous write-read of a single 16-bit word (utility/test)
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
      return true;
    }
    return false;
  }

  // True if TX FIFO has space
  static bool is_transmission_ready()
  {
    return tx_fifo_not_full();
  }

  // 2: data in FIFO, 1: FIFO full, 0: FIFO has space/empty
  static uint8_t get_queue_status()
  {
    // 2: data in FIFO (transmitting/ready), 1: FIFO full, 0: FIFO has space/empty
    if (!tx_fifo_empty()) return 2;
    if (!tx_fifo_not_full()) return 1;
    return 0;
  }

  // Queue a LOGAN bulk transfer (header + payload + checksum) if not already queued/active
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

  // Cancel any queued or active LOGAN bulk transfer immediately
  static inline void cancel_logan_transfer()
  {
    uint32_t status = save_and_disable_interrupts();
    // Remove any queued TX_BULK items
    if (txq_count > 0)
    {
      // Compact queue by skipping TX_BULK items
      uint32_t write_pos = txq_tail;
      uint32_t new_count = 0;
      for (uint32_t i = 0; i < txq_count; ++i)
      {
        uint32_t idx = (txq_tail + i) & (TXQ_CAPACITY - 1);
        if (txq_type[idx] != TX_BULK)
        {
          // keep events by moving to new_tail if necessary
          if (idx != write_pos)
          {
            txq_type[write_pos] = txq_type[idx];
            txq_word[write_pos] = txq_word[idx];
          }
          write_pos = (write_pos + 1) & (TXQ_CAPACITY - 1);
          ++new_count;
        }
      }
      // Tail remains the same; head advances to write_pos
      txq_head = write_pos;
      txq_count = new_count;
    }

    // Abort active bulk playback
    logan_queue_active = false;
    logan_queued = false;
    logan_queue_index = 0;
    logan_total_words = 0;
    logan_payload_words = 0;
    logan_checksum_accum = 0;
    restore_interrupts(status);
  }

  // Reset software state and reinitialize the SPI slave/ISR
  static inline void reset_state()
  {
    uint32_t status = save_and_disable_interrupts();
    // Disable SPI IRQ to prevent ISR during reset
    irq_set_enabled(SPI1_IRQ, false);

    // Deinit peripheral to clear FIFOs, then re-init fresh
    spi_deinit(spi1);

    // Clear software queues/state
    txq_head = 0;
    txq_tail = 0;
    txq_count = 0;
    logan_queue_index = 0;
    logan_queue_active = false;
    logan_queued = false;
    logan_header_word = 0;
    logan_checksum_accum = 0;
    logan_payload_words = 0;
    logan_total_words = 0;

    restore_interrupts(status);

    // Re-initialize SPI slave and ISR; leaves clean TX/RX FIFOs
    init_slave();
  }

  // True if any dummy samples remain to transmit in the active bulk
  static inline bool has_dummy_samples_pending()
  {
    return logan_queue_active && (logan_queue_index < logan_total_words);
  }

  // Fetch next word of active bulk (header, payload, then checksum)
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
    size_t payload_index = logan_queue_index - 1;
    uint16_t sample = 0;
    if (payload_index < logan_payload_words)
    {
      sample = logan_payload_storage[payload_index];
    }
    logan_checksum_accum ^= sample;
    out = sample;
    logan_queue_index++;
    return true;
  }

  // Accessors for nibble mappings (external configuration)
  static inline size_t samples_from_nibble(uint8_t samples_nibble)
  {
    return map_samples_nibble(samples_nibble);
  }

  static inline size_t rate_from_nibble(uint8_t rate_nibble)
  {
    return map_sampling_rate_nibble(rate_nibble);
  }

  // Copy nibble-packed sample payload into internal storage for streaming
  static inline void set_logan_payload(const uint16_t *data, size_t count)
  {
    if (count > logan_payload_words) count = logan_payload_words;
    if (count > LOGAN_MAX_SAMPLES) count = LOGAN_MAX_SAMPLES;
    for (size_t i = 0; i < count; ++i)
    {
      logan_payload_storage[i] = data[i];
    }
  }

  // Directly configure header and payload length (for multi-channel custom headers)
  static inline void configure_custom_header(uint16_t header_word, size_t payload_words)
  {
    logan_header_word = header_word;
    logan_payload_words = payload_words;
    if (logan_payload_words > LOGAN_MAX_SAMPLES) logan_payload_words = LOGAN_MAX_SAMPLES;
    logan_total_words = 1 /*header*/ + logan_payload_words + 1 /*checksum*/;
    logan_checksum_accum = 0;
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
uint16_t SPIComm::logan_payload_storage[SPIComm::LOGAN_MAX_SAMPLES] = {};

#endif

// EOF
