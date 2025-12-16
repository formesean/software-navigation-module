#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>
#include <stdio.h>
#include <cstdint>
#include <cstdlib>

#include "ring_buffer.hpp"
#include "spi_comm.hpp"

// Pin Definitions
const uint8_t LOGAN_PINS[4] = {16, 18, 20, 22};
const uint8_t MACRO_KEYS[5] = {5, 6, 7, 8, 9};
const uint8_t ENCODER_SW = 2;
const uint8_t ENCODER_DT = 3;
const uint8_t ENCODER_CLK = 4;
const uint8_t LED_PIN = 25;

// Global state
RingBuffer ringBuffer;

volatile bool led_blink_requested = false;
volatile int32_t encoder_position = 0;
volatile uint8_t encoder_last_state = 0;

volatile uint16_t g_rx_word = 0;
volatile bool g_system_enabled = false;
volatile bool g_stop_requested = false;
volatile bool g_start_requested = false;
const uint16_t START_CMD = SPIComm::start_word();
const uint16_t STOP_CMD = SPIComm::stop_word();

// Debounce timings
const uint32_t ENCODER_DEBOUNCE_US = 3000;
const uint32_t ENCODER_SW_DEBOUNCE_US = 50000;
const uint32_t MACRO_KEY_DEBOUNCE_US = 20000;

// Quadrature encoder state transition table (last_state<<2 | current_state)
const int8_t encoder_transition_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1,
    -1, 0, 0, 1, 0, 1, -1, 0};

// Debounce timers
volatile absolute_time_t last_encoder_event = 0;
volatile absolute_time_t last_encoder_button_event = 0;

// Last-known input states and event flags
volatile bool prev_macro_state[5] = {true, true, true, true, true};
volatile absolute_time_t last_macro_debounce_time[5] = {0};
volatile bool prev_encoder_sw_state = true;

volatile bool macro_event_pending[5] = {false};
volatile bool encoder_sw_event_pending = false;

static inline bool control_allows_packets()
{
  return g_system_enabled && !g_stop_requested;
}

// Function prototypes
void wait_for_usb_connect();
void setup_pins();
void request_led_blink();
void shared_irq_handler();
void process_buffered_events();
void initialize_pin_states();
void reset_system_state();
bool is_snm_event(uint16_t word);
void handle_snm_announcement(uint16_t word);
void logan_trigger_handler();

// LOGAN: sampling timer and raw sample buffers (per-channel)
static repeating_timer_t g_logan_timer;
static volatile bool g_logan_sampling_active = false;
static volatile bool g_logan_sampling_done = false;
static volatile size_t g_logan_sample_target = 0;
static volatile size_t g_logan_requested_samples = 0;
static volatile size_t g_logan_sample_index = 0;
static uint16_t g_logan_sample_buffer[4][2000];
static volatile uint8_t g_logan_samples_nibble = 0;
static volatile uint8_t g_logan_rate_nibble = 0;
static volatile bool g_logan_continuous = false;
static volatile size_t g_logan_expected_words = 0;
static volatile bool g_logan_abort_requested = false;

// LOGAN: multi-channel transmit sequencing state
static volatile bool g_logan_tx_sequence_active = false;
static volatile uint8_t g_logan_tx_next_channel = 1;

// LOGAN: minimal timing/tx state (debug logging removed)
static volatile absolute_time_t g_logan_sampling_start_time = 0;
static volatile absolute_time_t g_logan_sampling_end_time = 0;
static volatile bool g_logan_tx_inflight = false;
static volatile uint8_t g_logan_tx_inflight_channel = 0;

// LOGAN: trigger configuration/state
static volatile bool g_logan_trigger_armed = false;
static volatile uint8_t g_logan_trigger_mode = 0;
static volatile uint8_t g_logan_trigger_channel = 0;
static volatile bool g_logan_trigger_edge_rising = true;
static volatile bool g_logan_trigger_edge_falling = true;
static volatile bool g_logan_trigger_level_low = false;
static volatile bool g_logan_trigger_level_high = false;

// LOGAN: pre-trigger capture state
static constexpr size_t PRETRIGGER_MAX = 1000;
static volatile size_t g_logan_pre_capacity = 0;
static volatile size_t g_logan_pre_count = 0;
static volatile size_t g_logan_pre_wr_index = 0;
static uint8_t g_logan_pre_ring[4][PRETRIGGER_MAX] = {};
static volatile bool g_logan_triggered = false;
static volatile size_t g_logan_trigger_index = 0;
static volatile bool g_logan_prev_trig_level = false;

// Harden payload words so they cannot be misread as SNM events by the host.
// If top nibble is 0x1 and checksum nibble equals type^action^value, flip bit 1
// of the checksum nibble (keep LSB sample bit) to break equality.
static inline uint16_t harden_payload_word(uint16_t w)
{
  uint8_t nib3 = static_cast<uint8_t>((w >> 12) & 0x0F);
  if (nib3 == 0x1)
  {
    uint8_t nib2 = static_cast<uint8_t>((w >> 8) & 0x0F);
    uint8_t nib1 = static_cast<uint8_t>((w >> 4) & 0x0F);
    uint8_t nib0 = static_cast<uint8_t>(w & 0x0F);
    uint8_t checksum = static_cast<uint8_t>((nib3 ^ nib2 ^ nib1) & 0x0F);
    if (nib0 == checksum)
    {
      // Flip bit 1 to break checksum equality while preserving the LSB sample bit
      nib0 ^= 0x2;
      w = static_cast<uint16_t>(((nib3 & 0x0F) << 12) |
                                ((nib2 & 0x0F) << 8)  |
                                ((nib1 & 0x0F) << 4)  |
                                (nib0 & 0x0F));
    }
  }
  return w;
}

// Encode a 1-bit sample into a 4-bit nibble that preserves the LSB as the sample value
// but avoids colliding with SNM event type nibbles (1..3, 0xA) in the top nibble.
// Mapping: 0 -> 0x0, 1 -> 0x9 (1001b). Host should read (nibble & 0x1).
static inline uint16_t encode_sample_nibble(uint16_t sample)
{
  return (sample & 0x1) ? 0x5 : 0x0;
}

// Pack raw 1-bit samples (0/1) into 16-bit words (nibbles), MSB-first per word
static size_t pack_samples_to_words(const uint16_t *in_samples, size_t sample_count, uint16_t *out_words, size_t out_capacity)
{
  if (sample_count == 0) return 0;
  size_t word_count = (sample_count + 3) / 4;
  if (word_count > out_capacity) word_count = out_capacity;

  size_t remainder = sample_count % 4;
  size_t read_index = 0;
  size_t write_index = 0;

  if (remainder != 0 && write_index < word_count)
  {
    uint16_t w = 0;
    for (size_t i = 0; i < remainder && read_index < sample_count; ++i)
    {
      uint16_t nibble = encode_sample_nibble(static_cast<uint16_t>(in_samples[read_index++] & 0x1));
      size_t nib_index = (4 - remainder) + i; // place remainder at high nibbles
      w |= static_cast<uint16_t>(nibble << (nib_index * 4));
    }
    w = harden_payload_word(w);
    out_words[write_index++] = w;
  }

  while (write_index < word_count && read_index < sample_count)
  {
    uint16_t w = 0;
    for (size_t i = 0; i < 4 && read_index < sample_count; ++i)
    {
      uint16_t nibble = encode_sample_nibble(static_cast<uint16_t>(in_samples[read_index++] & 0x1));
      size_t nib_index = 3 - i; // MSB-first in each word
      w |= static_cast<uint16_t>(nibble << (nib_index * 4));
    }
    w = harden_payload_word(w);
    out_words[write_index++] = w;
  }

  return write_index;
}

// Capture one sampling tick immediately (all channels). Updates indices and
// marks completion when target reached. Safe to call from ISR or main context.
static inline void logan_capture_sample_now()
{
  if (!g_logan_sampling_active) return;

  if (g_logan_sample_index >= g_logan_sample_target)
  {
    g_logan_sampling_active = false;
    g_logan_sampling_done = true;
    g_logan_sampling_end_time = get_absolute_time();
    return;
  }
  uint32_t all_pins = gpio_get_all();

  // Extract samples for each channel using the pin definitions
  g_logan_sample_buffer[0][g_logan_sample_index] = (all_pins >> LOGAN_PINS[0]) & 0x1;
  g_logan_sample_buffer[1][g_logan_sample_index] = (all_pins >> LOGAN_PINS[1]) & 0x1;
  g_logan_sample_buffer[2][g_logan_sample_index] = (all_pins >> LOGAN_PINS[2]) & 0x1;
  g_logan_sample_buffer[3][g_logan_sample_index] = (all_pins >> LOGAN_PINS[3]) & 0x1;

  g_logan_sample_index++;

  if (g_logan_sample_index >= g_logan_sample_target)
  {
    g_logan_sampling_active = false;
    g_logan_sampling_done = true;
    g_logan_sampling_end_time = get_absolute_time();
  }
}

// LOGAN sampling timer callback
// Returns true to continue repeating, false to stop
bool logan_timer_callback(repeating_timer_t *rt)
{
  if (!g_logan_sampling_active) return false;

  // Snapshot all channels simultaneously by reading the entire GPIO register at once.
  uint32_t all_pins = gpio_get_all();
  uint8_t snapshot[4];
  snapshot[0] = (all_pins >> LOGAN_PINS[0]) & 0x1;
  snapshot[1] = (all_pins >> LOGAN_PINS[1]) & 0x1;
  snapshot[2] = (all_pins >> LOGAN_PINS[2]) & 0x1;
  snapshot[3] = (all_pins >> LOGAN_PINS[3]) & 0x1;


  // If pre-trigger enabled and not triggered yet, manage ring and check trigger
  if (!g_logan_triggered && g_logan_pre_capacity > 0)
  {
    // If ring not yet full, push snapshot and continue
    if (g_logan_pre_count < g_logan_pre_capacity)
    {
      for (uint8_t ch = 0; ch < 4; ++ch)
      {
        g_logan_pre_ring[ch][g_logan_pre_wr_index] = snapshot[ch];
      }
      if (g_logan_pre_count < g_logan_pre_capacity) g_logan_pre_count++;
      g_logan_pre_wr_index = (g_logan_pre_wr_index + 1) % g_logan_pre_capacity;
      g_logan_prev_trig_level = snapshot[g_logan_trigger_channel] != 0;
      return true;
    }

    // Ring full: evaluate trigger on the configured channel BEFORE pushing snapshot
    bool trig_now = snapshot[g_logan_trigger_channel] != 0;
    bool fire = false;
    if (g_logan_trigger_edge_rising && !g_logan_prev_trig_level && trig_now) fire = true;
    if (g_logan_trigger_edge_falling && g_logan_prev_trig_level && !trig_now) fire = true;
    if (g_logan_trigger_level_high && trig_now) fire = true;
    if (g_logan_trigger_level_low && !trig_now) fire = true;

    if (!g_logan_trigger_armed && g_logan_trigger_mode != 0)
    {
      // keep armed state coherent
      g_logan_trigger_armed = true;
    }

    if (g_logan_trigger_armed && fire)
    {
      // Copy exactly the pre-trigger window from ring (oldest first)
      size_t pre_to_copy = g_logan_pre_capacity;
      size_t start_idx = (g_logan_pre_wr_index + (g_logan_pre_capacity - pre_to_copy)) % g_logan_pre_capacity;
      size_t dst = 0;
      for (size_t i = 0; i < pre_to_copy && dst < g_logan_sample_target; ++i)
      {
        size_t src = (start_idx + i) % g_logan_pre_capacity;
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
          g_logan_sample_buffer[ch][dst] = g_logan_pre_ring[ch][src];
        }
        dst++;
      }

      // Place the trigger sample itself from current snapshot
      if (dst < g_logan_sample_target)
      {
        for (uint8_t ch = 0; ch < 4; ++ch)
          g_logan_sample_buffer[ch][dst] = snapshot[ch];
        g_logan_trigger_index = dst;
        dst++;
      }

      // Switch to post-trigger capture state; do not push this snapshot to ring
      g_logan_triggered = true;
      g_logan_trigger_armed = false;
      g_logan_sample_index = dst;
      g_logan_prev_trig_level = trig_now;

      // Skip generic storage for this tick to avoid duplicating the trigger sample
      // Continue timer to collect post-trigger samples on subsequent ticks
      // Return here so the below generic storage block is bypassed for this cycle
      // (completion check will occur on a later tick)
      return true;
    }
    else if (!g_logan_triggered && g_logan_trigger_mode == 0)
    {
      // Immediate mode: treat as triggered on first tick with ring full
      size_t dst = 0;
      size_t pre_to_copy = g_logan_pre_capacity;
      size_t start_idx = (g_logan_pre_wr_index + (g_logan_pre_capacity - pre_to_copy)) % g_logan_pre_capacity;
      for (size_t i = 0; i < pre_to_copy && dst < g_logan_sample_target; ++i)
      {
        size_t src = (start_idx + i) % g_logan_pre_capacity;
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
          g_logan_sample_buffer[ch][dst] = g_logan_pre_ring[ch][src];
        }
        dst++;
      }
      if (dst < g_logan_sample_target)
      {
        for (uint8_t ch = 0; ch < 4; ++ch)
          g_logan_sample_buffer[ch][dst] = snapshot[ch];
        g_logan_trigger_index = dst;
        dst++;
      }
      g_logan_triggered = true;
      g_logan_sample_index = dst;
      g_logan_prev_trig_level = trig_now;
      return true;
    }
    else
    {
      // Not fired this tick: push snapshot into ring and continue
      for (uint8_t ch = 0; ch < 4; ++ch)
      {
        g_logan_pre_ring[ch][g_logan_pre_wr_index] = snapshot[ch];
      }
      g_logan_pre_wr_index = (g_logan_pre_wr_index + 1) % g_logan_pre_capacity;
      g_logan_prev_trig_level = trig_now;
      return true;
    }
  }

  // If we are already triggered (or pre-trigger disabled), just store snapshot
  if (!(!g_logan_triggered && g_logan_pre_capacity > 0))
  {
    if (g_logan_sample_index < g_logan_sample_target)
    {
      // === MODIFICATION START ===
      // Store the simultaneous snapshot
      g_logan_sample_buffer[0][g_logan_sample_index] = snapshot[0];
      g_logan_sample_buffer[1][g_logan_sample_index] = snapshot[1];
      g_logan_sample_buffer[2][g_logan_sample_index] = snapshot[2];
      g_logan_sample_buffer[3][g_logan_sample_index] = snapshot[3];
      // === MODIFICATION END ===
      g_logan_sample_index++;
    }
  }

  // Check completion
  if (g_logan_sample_index >= g_logan_sample_target)
  {
    g_logan_sampling_active = false;
    g_logan_sampling_done = true;
    g_logan_sampling_end_time = get_absolute_time();
    return false;
  }

  return true;
}

// Program entry point: initializes IO, IRQs, SPI slave, and runs event loop
int main()
{
  stdio_init_all();
  wait_for_usb_connect();
  setup_pins();
  initialize_pin_states();
  SPIComm::init_slave();

  encoder_last_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);

  irq_set_exclusive_handler(IO_IRQ_BANK0, shared_irq_handler);
  irq_set_enabled(IO_IRQ_BANK0, true);

  // Prioritize GPIO over timer sampling to avoid missing encoder/macro events under LOGAN load
  irq_set_priority(IO_IRQ_BANK0, 0);       // Highest priority for GPIO bank
  irq_set_priority(TIMER_IRQ_0, 0xC0);     // Lower priority for timer alarms
  irq_set_priority(TIMER_IRQ_1, 0xC0);
  irq_set_priority(TIMER_IRQ_2, 0xC0);
  irq_set_priority(TIMER_IRQ_3, 0xC0);

  gpio_set_irq_enabled(ENCODER_CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENCODER_DT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENCODER_SW, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

  for (int i = 0; i < 5; ++i)
  {
    gpio_set_irq_enabled(MACRO_KEYS[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
  }

  while (true)
  {
    process_buffered_events();

    if (led_blink_requested && g_system_enabled)
    {
      led_blink_requested = false;
      gpio_put(LED_PIN, 1);
      busy_wait_us(50000);
      gpio_put(LED_PIN, 0);
    }

    sleep_us(100);
  }

  return 0;
}

void wait_for_usb_connect()
{
  absolute_time_t timeout = make_timeout_time_ms(5000);
  while (!stdio_usb_connected() && !time_reached(timeout))
    sleep_ms(10);

  sleep_ms(100);
}

// Initialize GPIO directions and pulls for macro keys, encoder, LOGAN inputs, and LED
void setup_pins()
{
  for (uint key : MACRO_KEYS)
  {
    gpio_init(key);
    gpio_set_dir(key, GPIO_IN);
    gpio_pull_up(key);
  }

  for (uint key : LOGAN_PINS)
  {
    gpio_init(key);
    gpio_set_dir(key, GPIO_IN);
    gpio_disable_pulls(key);
  }

  gpio_init(ENCODER_SW);
  gpio_set_dir(ENCODER_SW, GPIO_IN);
  gpio_pull_up(ENCODER_SW);

  gpio_init(ENCODER_DT);
  gpio_set_dir(ENCODER_DT, GPIO_IN);
  gpio_pull_up(ENCODER_DT);

  gpio_init(ENCODER_CLK);
  gpio_set_dir(ENCODER_CLK, GPIO_IN);
  gpio_pull_up(ENCODER_CLK);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);
}

void initialize_pin_states()
{
  sleep_ms(10);

  for (int i = 0; i < 5; ++i)
  {
    prev_macro_state[i] = gpio_get(MACRO_KEYS[i]);
  }
  prev_encoder_sw_state = gpio_get(ENCODER_SW);
}

void request_led_blink()
{
  led_blink_requested = true;
}

// Reset runtime state: SPI queues, buffers, input tracking, LOGAN sampling/trigger
void reset_system_state()
{
  SPIComm::reset_state();
  ringBuffer.clear();
  encoder_position = 0;
  encoder_last_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);
  for (int i = 0; i < 5; ++i)
  {
    prev_macro_state[i] = gpio_get(MACRO_KEYS[i]);
    last_macro_debounce_time[i] = 0;
    macro_event_pending[i] = false;
  }
  prev_encoder_sw_state = gpio_get(ENCODER_SW);
  last_encoder_event = 0;
  last_encoder_button_event = 0;
  encoder_sw_event_pending = false;
  led_blink_requested = false;
  gpio_put(LED_PIN, 0);

  // Reset LOGAN sampling state and stop timer if active
  g_logan_sampling_active = false;
  g_logan_sampling_done = false;
  g_logan_sample_index = 0;
  g_logan_sample_target = 0;
  g_logan_requested_samples = 0;
  g_logan_samples_nibble = 0;
  g_logan_rate_nibble = 0;
  g_logan_continuous = false;
  cancel_repeating_timer(&g_logan_timer);
  g_logan_tx_sequence_active = false;
  g_logan_tx_next_channel = 1;

  // Reset trigger state
  g_logan_trigger_armed = false;
  g_logan_trigger_mode = 0;
  g_logan_trigger_channel = 0;
  g_logan_trigger_edge_rising = true;
  g_logan_trigger_edge_falling = true;
  g_logan_trigger_level_low = false;
  g_logan_trigger_level_high = false;

  // Disable all LOGAN pin interrupts
  for (uint8_t ch = 0; ch < 4; ++ch)
  {
    gpio_set_irq_enabled(LOGAN_PINS[ch], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  }
}

// Shared GPIO IRQ handler: handles LOGAN trigger, macro keys, encoder rotation and switch
void shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();

  // Handle macro keys
  for (int i = 0; i < 5; ++i)
  {
    uint key_pin = MACRO_KEYS[i];
    uint32_t events = gpio_get_irq_event_mask(key_pin);

    if (events & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE))
    {
      gpio_acknowledge_irq(key_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);

      if (absolute_time_diff_us(last_macro_debounce_time[i], now) > MACRO_KEY_DEBOUNCE_US)
      {
        bool current_state = gpio_get(key_pin);
        bool current_pressed = !current_state;

        if (current_state != prev_macro_state[i])
        {
          prev_macro_state[i] = current_state;
          last_macro_debounce_time[i] = now;
          // Do not generate packets unless the system is running
          if (control_allows_packets())
          {
            uint16_t event_data = SPIComm::create_macro_key_event(i + 1, current_pressed);

            // Try to queue immediately; fallback to ring buffer if busy/full
            if (!SPIComm::queue_packet(event_data))
            {
              ringBuffer.push(event_data);
            }

            request_led_blink();
          }
        }
      }
    }
  }

  // Handle encoder rotation
  if (gpio_get_irq_event_mask(ENCODER_CLK) || gpio_get_irq_event_mask(ENCODER_DT))
  {
    gpio_acknowledge_irq(ENCODER_CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(ENCODER_DT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

    if (absolute_time_diff_us(last_encoder_event, now) > ENCODER_DEBOUNCE_US)
    {
      uint8_t current_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);

      if (current_state != encoder_last_state)
      {
        uint8_t transition_index = (encoder_last_state << 2) | current_state;
        int8_t delta = encoder_transition_table[transition_index & 0x0F];

        if (delta != 0)
        {
          encoder_position += delta;
          last_encoder_event = now;

          bool clockwise = delta > 0;
          uint8_t steps = static_cast<uint8_t>(abs(delta));

          if (control_allows_packets())
          {
            uint16_t event_data = SPIComm::create_encoder_rotate_event(clockwise, steps);

            // Try to queue immediately; fallback to ring buffer if busy/full
            if (!SPIComm::queue_packet(event_data))
            {
              ringBuffer.push(event_data);
            }

            request_led_blink();
          }
        }

        encoder_last_state = current_state;
      }
    }
  }

  // Handle encoder switch
  if (gpio_get_irq_event_mask(ENCODER_SW) & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE))
  {
    gpio_acknowledge_irq(ENCODER_SW, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);

    if (absolute_time_diff_us(last_encoder_button_event, now) > ENCODER_SW_DEBOUNCE_US)
    {
      bool current_state = gpio_get(ENCODER_SW);
      bool current_pressed = !current_state;

      if (current_state != prev_encoder_sw_state)
      {
        prev_encoder_sw_state = current_state;
        last_encoder_button_event = now;
        // Allow multiple events without waiting for main-loop acknowledgment

        if (control_allows_packets())
        {
          uint16_t event_data = SPIComm::create_encoder_switch_event(current_pressed);

          // Try to queue immediately; fallback to ring buffer if busy/full
          if (!SPIComm::queue_packet(event_data))
          {
            ringBuffer.push(event_data);
          }

          request_led_blink();
        }
      }
    }
  }
}

// Process deferred events, handle control commands, and manage LOGAN TX sequencing
void process_buffered_events()
{
  uint16_t event_data;
  static absolute_time_t last_process_time = 0;
  absolute_time_t now = get_absolute_time();

  SPIComm::update_transmission_status();

  SPIComm::ControlFlags control = SPIComm::consume_control_flags();

  if (control.stop)
  {
    g_system_enabled = false;
    reset_system_state();
    return;
  }

  if (!g_system_enabled && control.start)
  {
    g_system_enabled = true;
  }

  // Do not emit or process packets until a START has been received
  if (!g_system_enabled)
  {
    return;
  }

  uint16_t rx_snapshot = g_rx_word;
  if (rx_snapshot != 0)
  {
    uint8_t rx_high_byte = (rx_snapshot >> 8) & 0xFF;
    uint8_t rx_low_byte  =  rx_snapshot       & 0xFF;

    // Prefer LOGAN header detection first, so single-shot (MSB=0) is not misread as SNM
    bool msb_set = ((rx_high_byte & 0x80) != 0);
    bool header_nonzero = (rx_high_byte != 0x00 || rx_low_byte != 0x00);
    uint8_t type_nibble_peek = static_cast<uint8_t>((rx_high_byte >> 4) & 0x0F);
    uint8_t chan_peek = static_cast<uint8_t>(type_nibble_peek & 0x07);
    bool looks_like_logan = header_nonzero && (msb_set || (chan_peek >= 1 && chan_peek <= 4));
    bool is_logan_cmd = looks_like_logan;
    if (is_logan_cmd)
    {
      uint8_t type_nibble    = (rx_high_byte >> 4) & 0x0F; // bccc (b=MSB for mode, c = channel 1..4)
      uint8_t trig_mode_nib  =  rx_high_byte       & 0x0F;
      uint8_t samples_nibble = (rx_low_byte  >> 4) & 0x0F;
      uint8_t rate_nibble    =  rx_low_byte        & 0x0F;

      uint8_t trig_chan_1to4 = type_nibble & 0x07; // 1..4 expected (0 reserved)
      uint8_t trig_index = (trig_chan_1to4 >= 1 && trig_chan_1to4 <= 4) ? (uint8_t)(trig_chan_1to4 - 1) : 0;

      if (samples_nibble == 0x0 && rate_nibble == 0x0)
      {
        // LOGAN STOP
        g_logan_continuous = false;
        g_logan_sampling_active = false;
        g_logan_sampling_done = false;
        g_logan_abort_requested = true;
        cancel_repeating_timer(&g_logan_timer);
        g_logan_sample_index = 0;
        g_logan_sample_target = 0;
        g_logan_expected_words = 0;
        SPIComm::cancel_logan_transfer();
        g_logan_tx_sequence_active = false;
        g_logan_tx_next_channel = 1;
        g_rx_word = 0; // consume
      }
      else
      {
        // LOGAN START / (re-)arm
        size_t expected_samples = SPIComm::samples_from_nibble(samples_nibble);
        size_t max_samples = sizeof(g_logan_sample_buffer[0]) / sizeof(g_logan_sample_buffer[0][0]);
        size_t requested_samples = (expected_samples > max_samples) ? max_samples : expected_samples;
        size_t rate_hz     = SPIComm::rate_from_nibble(rate_nibble);
        if (rate_hz < 1) rate_hz = 1;
        int64_t interval_us = (int64_t)((1000000 + (rate_hz / 2)) / rate_hz);
        if (interval_us <= 0) interval_us = 1;

        cancel_repeating_timer(&g_logan_timer);

        g_logan_samples_nibble = samples_nibble;
        g_logan_rate_nibble    = rate_nibble;
        // Capture exactly the requested window length
        g_logan_sample_target = requested_samples;
        g_logan_requested_samples = requested_samples;
        g_logan_sample_index   = 0;
        g_logan_sampling_done  = false;
        // Continuous if MSB of type nibble set (cb_run_stop). Single-shot if MSB clear (cb_single)
        g_logan_continuous      = msb_set;
        g_logan_abort_requested = false; // clear any previous abort
        // Transmit only the requested window length, not the doubled capture
        g_logan_expected_words  = (requested_samples + 3) / 4;

        // Pre-trigger configuration:
        // - Immediate mode: pre-trigger = half window
        // - Single-shot triggered: enable pre-trigger = half window to center trigger
        // - Continuous: no pre-trigger
        if (trig_mode_nib == 0)
        {
          g_logan_pre_capacity = (requested_samples / 2);
          if (g_logan_pre_capacity > PRETRIGGER_MAX) g_logan_pre_capacity = PRETRIGGER_MAX;
        }
        else
        {
          // For triggered captures, choose pre-trigger so that the trigger sample
          // lands at index N/2 - 1 for even N (matching GUI 0 s placement)
          size_t pre = requested_samples / 2;
          if ((requested_samples % 2) == 0)
          {
            if (pre > 0) pre -= 1; // shift one left for even counts
          }

          if (pre == 0 && requested_samples > 0)
          {
              pre = 1;
          }

          g_logan_pre_capacity = pre;
          if (g_logan_pre_capacity > PRETRIGGER_MAX) g_logan_pre_capacity = PRETRIGGER_MAX;
        }
        g_logan_pre_count = 0;
        g_logan_pre_wr_index = 0;
        g_logan_triggered = false;
        g_logan_trigger_index = 0;
        g_logan_prev_trig_level = static_cast<bool>(gpio_get(LOGAN_PINS[trig_index]));

        // Configure trigger settings for both single-shot and continuous modes
        g_logan_trigger_mode = trig_mode_nib;
        g_logan_trigger_channel = trig_index; // 0-based index into LOGAN_PINS

        // Disable any existing trigger interrupts
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
          gpio_set_irq_enabled(LOGAN_PINS[ch], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        }

        g_logan_sampling_active = true;

        if (trig_mode_nib == 0)
        {
          // Mode 0: immediate start (no trigger)
          g_logan_trigger_armed = false;
          g_logan_sampling_start_time = get_absolute_time();
        }
        else
        {
          // Mode 1+: triggered start (pre-trigger ring handled in timer)
          g_logan_trigger_armed = true;

          // Configure trigger detection based on mode (for edge/level semantics)
          switch (trig_mode_nib)
          {
            case 1: // Low level
              g_logan_trigger_edge_rising = false;
              g_logan_trigger_edge_falling = false;
              g_logan_trigger_level_low = true;
              g_logan_trigger_level_high = false;
              break;
            case 2: // High level
              g_logan_trigger_edge_rising = false;
              g_logan_trigger_edge_falling = false;
              g_logan_trigger_level_low = false;
              g_logan_trigger_level_high = true;
              break;
            case 3: // Rising edge
              g_logan_trigger_edge_rising = true;
              g_logan_trigger_edge_falling = false;
              g_logan_trigger_level_low = false;
              g_logan_trigger_level_high = false;
              break;
            case 4: // Falling edge
              g_logan_trigger_edge_rising = false;
              g_logan_trigger_edge_falling = true;
              g_logan_trigger_level_low = false;
              g_logan_trigger_level_high = false;
              break;
            case 5: // Either edge
              g_logan_trigger_edge_rising = true;
              g_logan_trigger_edge_falling = true;
              g_logan_trigger_level_low = false;
              g_logan_trigger_level_high = false;
              break;
            default: // Default to rising edge
              g_logan_trigger_edge_rising = true;
              g_logan_trigger_edge_falling = false;
              g_logan_trigger_level_low = false;
              g_logan_trigger_level_high = false;
              break;
          }
        }

        // Start periodic sampling; trigger alignment happens inside the timer ISR
        add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);

        g_rx_word = 0; // consume
      }
    }
    else if (is_snm_event(rx_snapshot))
    {
      const uint8_t type = static_cast<uint8_t>((rx_snapshot >> 12) & 0x0F);
      if (type == 0xA) // SNM announcement
      {
        handle_snm_announcement(rx_snapshot);
      }
      // Other SNM events (macro keys, encoder) are handled by the existing event system
      g_rx_word = 0; // consume the event
      return;
    }
  }

  if (absolute_time_diff_us(last_process_time, now) < 1000)
  {
    return;
  }
  last_process_time = now;

  for (int i = 0; i < 5; ++i)
  {
    if (macro_event_pending[i] &&
        absolute_time_diff_us(last_macro_debounce_time[i], now) > MACRO_KEY_DEBOUNCE_US)
    {
      macro_event_pending[i] = false;
    }
  }

  if (encoder_sw_event_pending && absolute_time_diff_us(last_encoder_button_event, now) > ENCODER_SW_DEBOUNCE_US)
  {
    encoder_sw_event_pending = false;
  }

  int processed_count = 0;
  const int max_events_per_cycle = 2;

  while (ringBuffer.pop(event_data) && processed_count < max_events_per_cycle)
  {
    if (SPIComm::queue_packet(event_data))
    {
      processed_count++;
    }
    else
    {
      ringBuffer.push(event_data);
      break;
    }

    // Reduce per-event delay to minimize latency under load
    sleep_us(50);
  }

  // LOGAN multi-channel: when sampling finishes, queue channel packets sequentially (1..4)
  if (g_logan_sampling_done || (g_logan_tx_sequence_active && !SPIComm::is_logan_transfer_busy()))
  {
    // Handle abort before starting/continuing sequence
    if (g_logan_sampling_done && g_logan_abort_requested)
    {
      g_logan_sampling_done = false;
      g_logan_sample_index = 0;
      g_logan_sample_target = 0;
      g_logan_expected_words = 0;
      g_logan_tx_sequence_active = false;
      return;
    }

    // If sampling just completed and TX is idle, start sequence at channel 1
    if (g_logan_sampling_done && !SPIComm::is_logan_transfer_busy())
    {
      g_logan_sampling_done = false;
      g_logan_tx_sequence_active = true;
      g_logan_tx_next_channel = 1;
      // sampling complete
    }

    // If a sequence is active and TX is idle, send next channel
    if (g_logan_tx_sequence_active && !SPIComm::is_logan_transfer_busy())
    {
      if (g_logan_tx_next_channel >= 1 && g_logan_tx_next_channel <= 4)
      {
        uint8_t ch = static_cast<uint8_t>(g_logan_tx_next_channel - 1);

        // Determine window to transmit
        size_t available_samples = g_logan_sample_index;
        size_t window_len = (g_logan_requested_samples > 0) ? g_logan_requested_samples : available_samples;
        if (window_len > available_samples) window_len = available_samples;
        size_t start_idx = 0;
        // Single-shot: transmit the assembled buffer from index 0 exactly
        if (!g_logan_continuous)
        {
          start_idx = 0;
          if (window_len > available_samples) window_len = available_samples;
        }

        // Pack raw 0/1 samples from selected window into nibble-packed 16-bit words for this channel
        uint16_t packed_words[600];
        size_t packed_count = pack_samples_to_words(&g_logan_sample_buffer[ch][start_idx], window_len, packed_words, 600);
        size_t payload_words = g_logan_expected_words > 0 ? g_logan_expected_words : packed_count;
        if (payload_words > 600) payload_words = 600;
        for (size_t i = packed_count; i < payload_words; ++i) packed_words[i] = 0;

        // Build header per channel: [15:12]=1ccc where ccc = channel (1..4)
        uint8_t samples_nib = g_logan_samples_nibble;
        uint8_t rate_nib    = g_logan_rate_nibble;
        uint8_t chan_num    = g_logan_tx_next_channel; // 1..4
        uint8_t trig_mode   = 0;
        uint8_t type_nib    = static_cast<uint8_t>(0x8 | (chan_num & 0x07));
        uint16_t header_word = ((type_nib   & 0x0F) << 12) |
                               ((trig_mode  & 0x0F) << 8)  |
                               ((samples_nib& 0x0F) << 4)  |
                               ((rate_nib   & 0x0F) << 0);

        SPIComm::configure_custom_header(header_word, payload_words);
        SPIComm::set_logan_payload(packed_words, payload_words);

        g_logan_tx_inflight = true;
        g_logan_tx_inflight_channel = g_logan_tx_next_channel;
        SPIComm::start_dummy_samples_transfer();

        g_logan_tx_next_channel++;
      }
      else
      {
        // Finished all 4 channels
        g_logan_tx_sequence_active = false;

        // If continuous, re-arm sampling for next multi-channel block
        if (g_logan_continuous)
        {
          cancel_repeating_timer(&g_logan_timer);

          g_logan_sample_index = 0;
          g_logan_sampling_done = false;
          g_logan_triggered = false;
          g_logan_trigger_index = 0;
          g_logan_pre_count = 0;
          g_logan_pre_wr_index = 0;
          g_logan_prev_trig_level = static_cast<bool>(gpio_get(LOGAN_PINS[g_logan_trigger_channel]));

          const bool immediate_mode = (g_logan_trigger_mode == 0);
          g_logan_sampling_active = true;
          if (immediate_mode)
          {
            g_logan_trigger_armed = false;
            g_logan_sampling_start_time = get_absolute_time();
          }
          else
          {
            g_logan_trigger_armed = true;
            g_logan_sampling_start_time = 0;
          }

          size_t rate_hz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
          if (rate_hz < 1) rate_hz = 1;
          int64_t interval_us = (int64_t)((1000000 + (rate_hz / 2)) / rate_hz);
          if (interval_us <= 0) interval_us = 1;

          // Immediate mode without a pre-trigger window benefits from a zero-latency first sample
          if (immediate_mode && g_logan_pre_capacity == 0)
          {
            logan_capture_sample_now();
          }

          add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);
        }
        else
        {
          // Single-shot: ensure we do not capture more samples until a new command
          cancel_repeating_timer(&g_logan_timer);
          g_logan_sampling_active = false;
          g_logan_trigger_armed = false;
          // Disable trigger pin interrupts to avoid spurious retriggers
          uint8_t trigger_pin = LOGAN_PINS[g_logan_trigger_channel];
          gpio_set_irq_enabled(trigger_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
          // Reset indices; host must re-arm explicitly to capture again
          g_logan_sample_index = 0;
        }
      }
    }
  }

  // no periodic debug
}

// Validate whether a word matches the SNM event format (excludes LOGAN headers)
bool is_snm_event(uint16_t word)
{
  const uint8_t type = static_cast<uint8_t>((word >> 12) & 0x0F);
  if ((type & 0x8) != 0) return false; // exclude LOGAN headers

  // Check checksum per SNM packet format
  const uint8_t action   = static_cast<uint8_t>((word >> 8)  & 0x0F);
  const uint8_t value    = static_cast<uint8_t>((word >> 4)  & 0x0F);
  const uint8_t checksum = static_cast<uint8_t>( word        & 0x0F);
  const bool checksum_ok = (checksum == ((type ^ action ^ value) & 0x0F));

  // Accept SNM events: macro keys (1), encoder rotate (2), encoder switch (3), announcements (0xA)
  return checksum_ok && ((type >= 0x1 && type <= 0x3) || type == 0xA);
}

// Handle SNM announcement messages and send acknowledgments
void handle_snm_announcement(uint16_t word)
{
  const uint8_t type   = static_cast<uint8_t>((word >> 12) & 0x0F);
  const uint8_t action = static_cast<uint8_t>((word >> 8)  & 0x0F);
  const uint8_t value  = static_cast<uint8_t>((word >> 4)  & 0x0F);

  if (type == 0xA && action == 0x1) // SNM announcement
  {
    if (value == 0x1) // SNM attached
    {
      // Send acknowledgment
      uint16_t ack_response = SPIComm::create_snm_announce_event(true); // Echo back attached=true
      if (!SPIComm::queue_packet(ack_response))
      {
        ringBuffer.push(ack_response);
      }
      request_led_blink();
    }
    else if (value == 0x0) // Program stopping
    {
      // Send acknowledgment
      uint16_t ack_response = SPIComm::create_snm_announce_event(false); // Echo back attached=false
      if (!SPIComm::queue_packet(ack_response))
      {
        ringBuffer.push(ack_response);
      }
      request_led_blink();

      // Trigger system stop (same as STOP_CMD)
      SPIComm::signal_stop_request();
    }
  }
}

// Evaluate trigger conditions and, when met, start LOGAN sampling
void logan_trigger_handler()
{
  // Only if armed and not already sampling
  if (!g_logan_trigger_armed || g_logan_sampling_active)
    return;

  uint8_t trigger_pin = LOGAN_PINS[g_logan_trigger_channel];
  bool trigger_condition_met = false;

  // Edge-based triggers
  if (g_logan_trigger_edge_rising || g_logan_trigger_edge_falling)
  {
    uint32_t events = gpio_get_irq_event_mask(trigger_pin);

    if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL))
    {
      // Use IRQ event bits to classify edges precisely
      if ((events & GPIO_IRQ_EDGE_RISE) && g_logan_trigger_edge_rising)
        trigger_condition_met = true;
      if ((events & GPIO_IRQ_EDGE_FALL) && g_logan_trigger_edge_falling)
        trigger_condition_met = true;

      gpio_acknowledge_irq(trigger_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    }
  }
  // Level-based triggers
  else if (g_logan_trigger_level_low || g_logan_trigger_level_high)
  {
    bool current_state = gpio_get(trigger_pin);

    // Level conditions
    if (!current_state && g_logan_trigger_level_low)
      trigger_condition_met = true;
    else if (current_state && g_logan_trigger_level_high)
      trigger_condition_met = true;
  }

  if (trigger_condition_met)
  {
    // Start sampling
    g_logan_trigger_armed = false;
    g_logan_sampling_active = true;
    g_logan_sample_index = 0;
    g_logan_sampling_start_time = get_absolute_time();

    // Disable trigger pin interrupts temporarily
    gpio_set_irq_enabled(trigger_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Start the sampling timer
    size_t rate_hz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
    if (rate_hz < 1) rate_hz = 1;
    int64_t interval_us = (int64_t)((1000000 + (rate_hz / 2)) / rate_hz);
    if (interval_us <= 0) interval_us = 1;

    // Schedule sampling; pre-trigger logic in the timer will place the trigger sample
    // using the current tick's snapshot without duplicating
    add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);
  }
}

// EOF
