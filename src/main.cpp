#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/pwm.h>
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
const uint8_t PWM_PIN = 15;

// PWM Configuration for Logic Analyzer Testing
const float PWM_FREQ_HZ = 1000.0f;
const float PWM_DUTY_CYCLE = 0.5f;
const bool PWM_ALWAYS_ENABLED = true;

// PWM state tracking
static uint g_pwm_slice_num = 0;
static uint g_pwm_channel = 0;
static volatile bool g_pwm_initialized = false;

// State Variables
RingBuffer ringBuffer;

volatile bool led_blink_requested = false;
volatile int32_t encoder_position = 0;
volatile uint8_t encoder_last_state = 0;

volatile uint16_t g_rx_word = 0;
volatile bool g_system_enabled = false;
volatile bool g_stop_requested = false;
volatile bool g_start_requested = false;
const uint16_t START_CMD = 0xA11A;
const uint16_t STOP_CMD = 0xA10B;

// Timing constants
const uint32_t ENCODER_DEBOUNCE_US = 3000;
const uint32_t ENCODER_SW_DEBOUNCE_US = 20000;
const uint32_t MACRO_KEY_DEBOUNCE_US = 20000;

// Encoder state transition table
const int8_t encoder_transition_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1,
    -1, 0, 0, 1, 0, 1, -1, 0};

// Timing variables
volatile absolute_time_t last_encoder_event = 0;
volatile absolute_time_t last_encoder_button_event = 0;

// State tracking
volatile bool prev_macro_state[5] = {true, true, true, true, true};
volatile absolute_time_t last_macro_debounce_time[5] = {0};
volatile bool prev_encoder_sw_state = true;

volatile bool macro_event_pending[5] = {false};
volatile bool encoder_sw_event_pending = false;

// Function Prototypes
void wait_for_usb_connect();
void setup_pins();
void request_led_blink();
void shared_irq_handler();
void process_buffered_events();
void initialize_pin_states();
void setup_pwm(uint pin);
void reset_system_state();
bool is_snm_event(uint16_t word);
void handle_snm_announcement(uint16_t word);
void logan_trigger_handler();

// LOGAN sampling timer and buffers
static repeating_timer_t g_logan_timer;
static volatile bool g_logan_sampling_active = false;
static volatile bool g_logan_sampling_done = false;
static volatile size_t g_logan_sample_target = 0;
static volatile size_t g_logan_sample_index = 0;
static uint16_t g_logan_sample_buffer[4][2000];
static volatile uint8_t g_logan_samples_nibble = 0;
static volatile uint8_t g_logan_rate_nibble = 0;
static volatile bool g_logan_continuous = false;
static volatile size_t g_logan_expected_words = 0; // expected payload words derived from nibble
static volatile bool g_logan_abort_requested = false;

// Multi-channel LOGAN TX sequencing state
static volatile bool g_logan_tx_sequence_active = false;
static volatile uint8_t g_logan_tx_next_channel = 1; // 1..4

// LOGAN trigger state
static volatile bool g_logan_trigger_armed = false;
static volatile uint8_t g_logan_trigger_mode = 0;
static volatile uint8_t g_logan_trigger_channel = 0;
static volatile bool g_logan_trigger_edge_rising = true;
static volatile bool g_logan_trigger_edge_falling = true;
static volatile bool g_logan_trigger_level_low = false;
static volatile bool g_logan_trigger_level_high = false;

static constexpr bool LOGAN_DEBUG_PRINT = false;

// Ensure a payload word cannot be misinterpreted as an SNM event by the host.
// If the top nibble equals 0x1 (SNM type range risk), and the checksum nibble equals
// type^action^value, tweak bit 1 of the checksum nibble (LSB preserved) to break equality.
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

// Pack raw 1-bit samples (stored as 0/1) into 16-bit words per nibble rules
// First word contains remainder (N % 4) samples in low nibbles, then full words of 4 samples
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
      uint16_t nibble = static_cast<uint16_t>(in_samples[read_index++] & 0xF);
      w |= static_cast<uint16_t>(nibble << (i * 4));
    }
    w = harden_payload_word(w);
    out_words[write_index++] = w;
  }

  while (write_index < word_count && read_index < sample_count)
  {
    uint16_t w = 0;
    for (size_t i = 0; i < 4 && read_index < sample_count; ++i)
    {
      uint16_t nibble = static_cast<uint16_t>(in_samples[read_index++] & 0xF);
      w |= static_cast<uint16_t>(nibble << (i * 4));
    }
    w = harden_payload_word(w);
    out_words[write_index++] = w;
  }

  return write_index;
}

bool logan_timer_callback(repeating_timer_t *rt)
{
  if (!g_logan_sampling_active) return false;

  if (g_logan_sample_index >= g_logan_sample_target)
  {
    g_logan_sampling_active = false;
    g_logan_sampling_done = true;
    return false;
  }

  // Sample all channels this tick into per-channel buffers
  for (uint8_t ch = 0; ch < 4; ++ch)
  {
    uint16_t sample = static_cast<uint16_t>(gpio_get(LOGAN_PINS[ch]) & 0x1);
    g_logan_sample_buffer[ch][g_logan_sample_index] = sample;
  }
  g_logan_sample_index++;

  if (g_logan_sample_index >= g_logan_sample_target)
  {
    g_logan_sampling_active = false;
    g_logan_sampling_done = true;
    return false;
  }

  return true;
}

int main()
{
  stdio_init_all();
  wait_for_usb_connect();
  setup_pins();
  initialize_pin_states();
  setup_pwm(PWM_PIN);
  SPIComm::init_slave();

  encoder_last_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);

  irq_set_exclusive_handler(IO_IRQ_BANK0, shared_irq_handler);
  irq_set_enabled(IO_IRQ_BANK0, true);

  // Prioritize GPIO events over timer sampling to avoid missed encoder/macro events during LOGAN
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

  // Ensure PWM remains enabled for testing (always on)
  if (g_pwm_initialized)
  {
    pwm_set_enabled(g_pwm_slice_num, PWM_ALWAYS_ENABLED);
  }
}

void shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();

  // Handle LOGAN trigger first (highest priority)
  logan_trigger_handler();

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
          // Allow multiple events without waiting for main-loop acknowledgment

          uint16_t event_data = SPIComm::create_macro_key_event(i + 1, current_pressed);

          // Attempt immediate SPI queueing; fallback to ring buffer if busy/full
          if (!SPIComm::queue_packet(event_data))
          {
            ringBuffer.push(event_data);
          }

          request_led_blink();
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

          uint16_t event_data = SPIComm::create_encoder_rotate_event(clockwise, steps);

          // Attempt immediate SPI queueing; fallback to ring buffer if busy/full
          if (!SPIComm::queue_packet(event_data))
          {
            ringBuffer.push(event_data);
          }

          request_led_blink();
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

        uint16_t event_data = SPIComm::create_encoder_switch_event(current_pressed);

        // Attempt immediate SPI queueing; fallback to ring buffer if busy/full
        if (!SPIComm::queue_packet(event_data))
        {
          ringBuffer.push(event_data);
        }

        request_led_blink();
      }
    }
  }

}

void process_buffered_events()
{
  uint16_t event_data;
  static absolute_time_t last_process_time = 0;
  absolute_time_t now = get_absolute_time();

  SPIComm::update_transmission_status();

  // Handle control commands atomically here
  if (g_stop_requested)
  {
    g_stop_requested = false;
    g_system_enabled = false;
    reset_system_state();
    if (g_rx_word == STOP_CMD || g_rx_word == START_CMD) g_rx_word = 0;
    return;
  }

  if (!g_system_enabled && g_start_requested)
  {
    g_start_requested = false;
    g_system_enabled = true;
    if (g_rx_word == START_CMD) g_rx_word = 0;
  }

  uint16_t rx_snapshot = g_rx_word;
  if (rx_snapshot != 0)
  {
    if (rx_snapshot == STOP_CMD)
    {
      g_stop_requested = true;
      g_rx_word = 0;
      return;
    }
    else if (rx_snapshot == START_CMD)
    {
      g_start_requested = true;
      g_rx_word = 0;
    }
    else
    {
      // Check if this is an SNM event (including announcements)
      if (is_snm_event(rx_snapshot))
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

      uint8_t rx_high_byte = (rx_snapshot >> 8) & 0xFF;
      uint8_t rx_low_byte  =  rx_snapshot       & 0xFF;

      // New LOGAN header: top bit of top nibble set
      bool is_logan_cmd = ((rx_high_byte & 0x80) != 0);
      if (is_logan_cmd && (rx_high_byte != 0x00 || rx_low_byte != 0x00))
      {
        uint8_t type_nibble    = (rx_high_byte >> 4) & 0x0F; // 1ccc (c=channel)
        uint8_t trig_mode_nib  =  rx_high_byte       & 0x0F;
        uint8_t samples_nibble = (rx_low_byte  >> 4) & 0x0F;
        uint8_t rate_nibble    =  rx_low_byte        & 0x0F;

        uint8_t trig_channel   = type_nibble & 0x07; // 0..4

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
          printf("LOGAN STOP received; sampling and transfer cancelled\r\n");
          g_rx_word = 0; // consume
        }
        else
        {
          // LOGAN START / (re-)arm
          size_t expected_samples = SPIComm::samples_from_nibble(samples_nibble);
          size_t max_samples = sizeof(g_logan_sample_buffer[0]) / sizeof(g_logan_sample_buffer[0][0]);
          size_t sample_count = (expected_samples > max_samples) ? max_samples : expected_samples;
          size_t expected_words_clamped = (sample_count + 3) / 4;
          size_t rate_khz     = SPIComm::rate_from_nibble(rate_nibble);
          if (rate_khz == 0) rate_khz = 1;
          int64_t interval_us = (int64_t)(1000 / rate_khz);
          if (interval_us <= 0) interval_us = 1;

          cancel_repeating_timer(&g_logan_timer);

          g_logan_samples_nibble = samples_nibble;
          g_logan_rate_nibble    = rate_nibble;
          g_logan_sample_target  = sample_count;
          g_logan_sample_index   = 0;
          g_logan_sampling_done  = false;
          g_logan_continuous      = true; // until explicit LOGAN STOP
          g_logan_abort_requested = false; // clear any previous abort
          g_logan_expected_words  = expected_words_clamped;

          // Configure trigger based on trigger mode
          g_logan_trigger_mode = trig_mode_nib;
          g_logan_trigger_channel = trig_channel;

          // Disable any existing trigger interrupts
          for (uint8_t ch = 0; ch < 4; ++ch)
          {
            gpio_set_irq_enabled(LOGAN_PINS[ch], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
          }

          if (trig_mode_nib == 0)
          {
            // Mode 0: Immediate start (no trigger)
            g_logan_sampling_active = true;
            g_logan_trigger_armed = false;
            add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);
            printf("LOGAN ARM: ch=%u trig=%u (immediate) samples=%u words=%u rate_khz=%u interval_us=%lld\r\n",
                   (unsigned)trig_channel,
                   (unsigned)trig_mode_nib,
                   (unsigned)sample_count,
                   (unsigned)expected_words_clamped,
                   (unsigned)rate_khz,
                   (long long)interval_us);
          }
          else
          {
            // Mode 1+: Triggered start
            g_logan_sampling_active = false;
            g_logan_trigger_armed = true;

            // Configure trigger detection based on mode
            switch (trig_mode_nib)
            {
              case 1: // LOW level
                g_logan_trigger_edge_rising = false;
                g_logan_trigger_edge_falling = false;
                g_logan_trigger_level_low = true;
                g_logan_trigger_level_high = false;
                break;
              case 2: // HIGH level
                g_logan_trigger_edge_rising = false;
                g_logan_trigger_edge_falling = false;
                g_logan_trigger_level_low = false;
                g_logan_trigger_level_high = true;
                break;
              case 3: // RISING_EDGE
                g_logan_trigger_edge_rising = true;
                g_logan_trigger_edge_falling = false;
                g_logan_trigger_level_low = false;
                g_logan_trigger_level_high = false;
                break;
              case 4: // FALLING_EDGE
                g_logan_trigger_edge_rising = false;
                g_logan_trigger_edge_falling = true;
                g_logan_trigger_level_low = false;
                g_logan_trigger_level_high = false;
                break;
              case 5: // EITHER_EDGE
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

            // Setup trigger pin
            uint8_t trigger_pin = LOGAN_PINS[trig_channel];

            // For level triggers, check current state immediately
            if (g_logan_trigger_level_low || g_logan_trigger_level_high)
            {
              bool current_state = gpio_get(trigger_pin);
              bool trigger_now = false;

              if (!current_state && g_logan_trigger_level_low)
                trigger_now = true;
              else if (current_state && g_logan_trigger_level_high)
                trigger_now = true;

              if (trigger_now)
              {
                // Trigger immediately
                g_logan_trigger_armed = false;
                g_logan_sampling_active = true;
                g_logan_sample_index = 0;
                add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);

                printf("LOGAN ARM: ch=%u trig=%u (level immediate) samples=%u words=%u rate_khz=%u interval_us=%lld\r\n",
                       (unsigned)trig_channel,
                       (unsigned)trig_mode_nib,
                       (unsigned)sample_count,
                       (unsigned)expected_words_clamped,
                       (unsigned)rate_khz,
                       (long long)interval_us);
              }
              else
              {
                // Enable interrupts for level monitoring
                uint32_t trigger_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
                gpio_set_irq_enabled(trigger_pin, trigger_events, true);

                printf("LOGAN ARM: ch=%u trig=%u (level armed) samples=%u words=%u rate_khz=%u interval_us=%lld armed=1\r\n",
                       (unsigned)trig_channel,
                       (unsigned)trig_mode_nib,
                       (unsigned)sample_count,
                       (unsigned)expected_words_clamped,
                       (unsigned)rate_khz,
                       (long long)interval_us);
              }
            }
            else
            {
              // Edge triggers: enable appropriate edge interrupts
              uint32_t trigger_events = 0;
              if (g_logan_trigger_edge_rising) trigger_events |= GPIO_IRQ_EDGE_RISE;
              if (g_logan_trigger_edge_falling) trigger_events |= GPIO_IRQ_EDGE_FALL;
              gpio_set_irq_enabled(trigger_pin, trigger_events, true);

              printf("LOGAN ARM: ch=%u trig=%u (edge armed) samples=%u words=%u rate_khz=%u interval_us=%lld armed=1\r\n",
                     (unsigned)trig_channel,
                     (unsigned)trig_mode_nib,
                     (unsigned)sample_count,
                     (unsigned)expected_words_clamped,
                     (unsigned)rate_khz,
                     (long long)interval_us);
            }
          }

          g_rx_word = 0; // consume
        }
      }
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

  // LOGAN multi-channel: when sampling finished, queue channel packets sequentially (1..4)
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

    // If just completed sampling and TX is idle, start sequence at channel 1
    if (g_logan_sampling_done && !SPIComm::is_logan_transfer_busy())
    {
      g_logan_sampling_done = false;
      g_logan_tx_sequence_active = true;
      g_logan_tx_next_channel = 1;
    }

    // If a sequence is active and TX is idle, send next channel
    if (g_logan_tx_sequence_active && !SPIComm::is_logan_transfer_busy())
    {
      if (g_logan_tx_next_channel >= 1 && g_logan_tx_next_channel <= 4)
      {
        uint8_t ch = static_cast<uint8_t>(g_logan_tx_next_channel - 1);

        // Pack raw 0/1 samples into nibble-packed 16-bit words for this channel
        uint16_t packed_words[600];
        size_t packed_count = pack_samples_to_words(g_logan_sample_buffer[ch], g_logan_sample_index, packed_words, 600);
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

        if (LOGAN_DEBUG_PRINT)
        {
          printf("LOGAN CH%u HEADER: 0x%04X words=%u samples=%u\r\n",
                 (unsigned)chan_num,
                 header_word,
                 (unsigned)payload_words,
                 (unsigned)g_logan_sample_index);
        }

        SPIComm::set_logan_payload(packed_words, payload_words);
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

          if (g_logan_trigger_mode == 0)
          {
            // Immediate mode: start sampling right away
            g_logan_sampling_active = true;
            size_t rate_khz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
            if (rate_khz == 0) rate_khz = 1;
            int64_t interval_us = static_cast<int64_t>(1000 / rate_khz);
            if (interval_us <= 0) interval_us = 1;
            add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);
            if (LOGAN_DEBUG_PRINT)
            {
              printf("LOGAN RE-ARM: immediate mode, rate_khz=%u interval_us=%lld\r\n",
                     (unsigned)rate_khz,
                     (long long)interval_us);
            }
          }
          else
          {
            // Triggered mode: re-arm trigger
            g_logan_sampling_active = false;
            g_logan_trigger_armed = true;

            uint8_t trigger_pin = LOGAN_PINS[g_logan_trigger_channel];

            // For level triggers, check current state immediately
            if (g_logan_trigger_level_low || g_logan_trigger_level_high)
            {
              bool current_state = gpio_get(trigger_pin);
              bool trigger_now = false;

              if (!current_state && g_logan_trigger_level_low)
                trigger_now = true;
              else if (current_state && g_logan_trigger_level_high)
                trigger_now = true;

              if (trigger_now)
              {
                // Trigger immediately
                g_logan_trigger_armed = false;
                g_logan_sampling_active = true;
                g_logan_sample_index = 0;

                size_t rate_khz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
                if (rate_khz == 0) rate_khz = 1;
                int64_t interval_us = static_cast<int64_t>(1000 / rate_khz);
                if (interval_us <= 0) interval_us = 1;
                add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);

                if (LOGAN_DEBUG_PRINT)
                {
                  printf("LOGAN RE-ARM: level trigger immediate, channel=%u mode=%u\r\n",
                         (unsigned)g_logan_trigger_channel + 1,
                         (unsigned)g_logan_trigger_mode);
                }
              }
              else
              {
                // Enable interrupts for level monitoring
                uint32_t trigger_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
                gpio_set_irq_enabled(trigger_pin, trigger_events, true);

                if (LOGAN_DEBUG_PRINT)
                {
                  printf("LOGAN RE-ARM: level trigger armed, channel=%u mode=%u\r\n",
                         (unsigned)g_logan_trigger_channel + 1,
                         (unsigned)g_logan_trigger_mode);
                }
              }
            }
            else
            {
              // Edge triggers: re-enable trigger pin interrupts
              uint32_t trigger_events = 0;
              if (g_logan_trigger_edge_rising) trigger_events |= GPIO_IRQ_EDGE_RISE;
              if (g_logan_trigger_edge_falling) trigger_events |= GPIO_IRQ_EDGE_FALL;
              gpio_set_irq_enabled(trigger_pin, trigger_events, true);

              if (LOGAN_DEBUG_PRINT)
              {
                printf("LOGAN RE-ARM: edge trigger armed, channel=%u mode=%u\r\n",
                       (unsigned)g_logan_trigger_channel + 1,
                       (unsigned)g_logan_trigger_mode);
              }
            }
          }
        }
      }
    }
  }

  static absolute_time_t last_debug_time = 0;
  if (absolute_time_diff_us(last_debug_time, now) > 5000000)
  {
    last_debug_time = now;
  }
}

void setup_pwm(uint pin)
{
  // Configure GPIO for PWM function
  gpio_set_function(pin, GPIO_FUNC_PWM);

  // Get PWM slice and channel numbers
  g_pwm_slice_num = pwm_gpio_to_slice_num(pin);
  g_pwm_channel = pwm_gpio_to_channel(pin);

  // Calculate optimal PWM parameters for logic analyzer testing
  uint32_t sys_clk = clock_get_hz(clk_sys);  // System clock frequency (~125 MHz)

  // Use a reasonable divider for stable operation
  // Target: good resolution while avoiding overflow
  float divider = 125.0f;  // 125 MHz / 125 = 1 MHz PWM base frequency

  // Calculate wrap value for desired frequency
  uint32_t wrap = (uint32_t)((sys_clk / (divider * PWM_FREQ_HZ)) - 1);

  // Ensure wrap value is reasonable (minimum 1, maximum 65535)
  if (wrap < 1) wrap = 1;
  if (wrap > 65535) wrap = 65535;

  // Set PWM configuration
  pwm_set_clkdiv(g_pwm_slice_num, divider);
  pwm_set_wrap(g_pwm_slice_num, wrap);

  // Set duty cycle (50% for clean clock signal)
  uint32_t level = (uint32_t)(wrap * PWM_DUTY_CYCLE);
  pwm_set_chan_level(g_pwm_slice_num, g_pwm_channel, level);

  // Always enable PWM for consistent testing
  pwm_set_enabled(g_pwm_slice_num, PWM_ALWAYS_ENABLED);

  g_pwm_initialized = true;

  printf("PWM initialized for LOGAN testing:\r\n");
  printf("  Pin: %u, Slice: %u, Channel: %u\r\n", pin, g_pwm_slice_num, g_pwm_channel);
  printf("  Frequency: %.1f Hz, Divider: %.1f, Wrap: %u, Level: %u\r\n",
         PWM_FREQ_HZ, divider, wrap, level);
  printf("  Duty Cycle: %.1f%%, Always Enabled: %s\r\n",
         PWM_DUTY_CYCLE * 100.0f, PWM_ALWAYS_ENABLED ? "YES" : "NO");
}

bool is_snm_event(uint16_t word)
{
  const uint8_t type = static_cast<uint8_t>((word >> 12) & 0x0F);
  if ((type & 0x8) != 0) return false; // exclude LOGAN headers

  // Validate checksum against SNM packet format
  const uint8_t action   = static_cast<uint8_t>((word >> 8)  & 0x0F);
  const uint8_t value    = static_cast<uint8_t>((word >> 4)  & 0x0F);
  const uint8_t checksum = static_cast<uint8_t>( word        & 0x0F);
  const bool checksum_ok = (checksum == ((type ^ action ^ value) & 0x0F));

  // Accept SNM events: macro keys (1), encoder rotate (2), encoder switch (3), announcements (0xA)
  return checksum_ok && ((type >= 0x1 && type <= 0x3) || type == 0xA);
}

void handle_snm_announcement(uint16_t word)
{
  const uint8_t type   = static_cast<uint8_t>((word >> 12) & 0x0F);
  const uint8_t action = static_cast<uint8_t>((word >> 8)  & 0x0F);
  const uint8_t value  = static_cast<uint8_t>((word >> 4)  & 0x0F);

  if (type == 0xA && action == 0x1) // SNM announcement
  {
    if (value == 0x1) // SNM attached
    {
      printf("SNM ANNOUNCE: Master attached\r\n");

      // Send acknowledgment response
      uint16_t ack_response = SPIComm::create_snm_announce_event(true); // Echo back attached=true
      if (!SPIComm::queue_packet(ack_response))
      {
        ringBuffer.push(ack_response);
      }
      request_led_blink();
    }
    else if (value == 0x0) // Program stopping
    {
      printf("SNM ANNOUNCE: Master stopping\r\n");

      // Send acknowledgment response
      uint16_t ack_response = SPIComm::create_snm_announce_event(false); // Echo back attached=false
      if (!SPIComm::queue_packet(ack_response))
      {
        ringBuffer.push(ack_response);
      }
      request_led_blink();

      // Trigger system stop - same as receiving STOP_CMD
      g_stop_requested = true;
    }
  }
}

void logan_trigger_handler()
{
  // Check if we're armed and waiting for trigger
  if (!g_logan_trigger_armed || g_logan_sampling_active)
    return;

  uint8_t trigger_pin = LOGAN_PINS[g_logan_trigger_channel];
  bool trigger_condition_met = false;

  // Handle edge-based triggers
  if (g_logan_trigger_edge_rising || g_logan_trigger_edge_falling)
  {
    uint32_t events = gpio_get_irq_event_mask(trigger_pin);

    if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL))
    {
      gpio_acknowledge_irq(trigger_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

      bool current_state = gpio_get(trigger_pin);

      // Check trigger edge conditions
      if (current_state && g_logan_trigger_edge_rising)
        trigger_condition_met = true;
      else if (!current_state && g_logan_trigger_edge_falling)
        trigger_condition_met = true;
    }
  }
  // Handle level-based triggers
  else if (g_logan_trigger_level_low || g_logan_trigger_level_high)
  {
    bool current_state = gpio_get(trigger_pin);

    // Check trigger level conditions
    if (!current_state && g_logan_trigger_level_low)
      trigger_condition_met = true;
    else if (current_state && g_logan_trigger_level_high)
      trigger_condition_met = true;
  }

  if (trigger_condition_met)
  {
    // Start sampling immediately
    g_logan_trigger_armed = false;
    g_logan_sampling_active = true;
    g_logan_sample_index = 0;

    // Disable trigger pin interrupts temporarily
    gpio_set_irq_enabled(trigger_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Start the sampling timer
    size_t rate_khz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
    if (rate_khz == 0) rate_khz = 1;
    int64_t interval_us = static_cast<int64_t>(1000 / rate_khz);
    if (interval_us <= 0) interval_us = 1;

    add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);

    printf("LOGAN TRIGGER: Channel %u triggered (mode=%u), starting sampling at %u kHz\r\n",
           g_logan_trigger_channel + 1, g_logan_trigger_mode, (unsigned)rate_khz);
  }
}

// EOF
