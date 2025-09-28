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

const float PWM_FREQ_HZ = 500.0f;

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

// LOGAN sampling timer and buffers
static repeating_timer_t g_logan_timer;
static volatile bool g_logan_sampling_active = false;
static volatile bool g_logan_sampling_done = false;
static volatile size_t g_logan_sample_target = 0;
static volatile size_t g_logan_sample_index = 0;
static uint16_t g_logan_sample_buffer[2000];
static volatile uint8_t g_logan_samples_nibble = 0;
static volatile uint8_t g_logan_rate_nibble = 0;
static volatile bool g_logan_continuous = false;

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

  // Sample first channel (digital read); extend to multi-channel if needed
  uint16_t sample = static_cast<uint16_t>(gpio_get(LOGAN_PINS[0]) & 0x1);
  g_logan_sample_buffer[g_logan_sample_index++] = sample;

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
}

void shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();
  uint32_t irq_status = save_and_disable_interrupts();

  if (!g_system_enabled)
  {
    for (int i = 0; i < 5; ++i)
    {
      gpio_acknowledge_irq(MACRO_KEYS[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
    }
    gpio_acknowledge_irq(ENCODER_CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(ENCODER_DT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(ENCODER_SW, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
    restore_interrupts(irq_status);
    return;
  }

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

        if (current_state != prev_macro_state[i] && !macro_event_pending[i])
        {
          prev_macro_state[i] = current_state;
          last_macro_debounce_time[i] = now;
          macro_event_pending[i] = true;

          uint16_t event_data = SPIComm::create_macro_key_event(i + 1, current_pressed);

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

      if (current_state != prev_encoder_sw_state && !encoder_sw_event_pending)
      {
        prev_encoder_sw_state = current_state;
        last_encoder_button_event = now;
        encoder_sw_event_pending = true;

        uint16_t event_data = SPIComm::create_encoder_switch_event(current_pressed);

        if (!SPIComm::queue_packet(event_data))
        {
          ringBuffer.push(event_data);
        }

        request_led_blink();
      }
    }
  }

  restore_interrupts(irq_status);
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
      g_rx_word = 0; // consume
      return;
    }
    else if (rx_snapshot == START_CMD)
    {
      g_start_requested = true;
      g_rx_word = 0; // consume
    }
    else
    {
      uint8_t rx_high_byte = (rx_snapshot >> 8) & 0xFF;
      uint8_t rx_low_byte  =  rx_snapshot       & 0xFF;

      bool is_logan_cmd = ((rx_high_byte >> 4) == 0x06);
      if (is_logan_cmd && rx_high_byte != 0x00 && rx_low_byte != 0x00)
      {
        uint8_t type_nibble    = (rx_high_byte >> 4) & 0x0F; // 0x6
        uint8_t samples_nibble =  rx_high_byte       & 0x0F;
        uint8_t rate_nibble    = (rx_low_byte  >> 4) & 0x0F;
        uint8_t rx_checksum    =  rx_low_byte        & 0x0F;

        uint8_t calc = (type_nibble ^ samples_nibble ^ rate_nibble) & 0x0F;
        if (calc == rx_checksum)
        {
          if (samples_nibble == 0x0 && rate_nibble == 0x0)
          {
            // LOGAN STOP
            g_logan_continuous = false;
            g_logan_sampling_active = false;
            cancel_repeating_timer(&g_logan_timer);
            printf("LOGAN STOP received; continuous disabled\r\n");
            g_rx_word = 0; // consume
          }
          else
          {
            // LOGAN START / (re-)arm
            size_t sample_count = SPIComm::samples_from_nibble(samples_nibble);
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
            g_logan_sampling_active = true;
            g_logan_continuous      = true; // until explicit LOGAN STOP

            add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);

            printf("LOGAN ARM: samples=%u rate_khz=%u interval_us=%lld continuous=1\r\n",
                   (unsigned)sample_count, (unsigned)rate_khz, (long long)interval_us);

            g_rx_word = 0; // consume
          }
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

    sleep_us(500);
  }

  // Dummy samples are now streamed by SPI TX FIFO preloading within SPIComm
  // If LOGAN sampling finished, prepare payload and start transfer
  if (g_logan_sampling_done)
  {
    // Only package and send when the previous bulk is not busy
    if (!SPIComm::is_logan_transfer_busy())
    {
      g_logan_sampling_done = false;

      // Pack raw 0/1 samples into nibble-packed 16-bit words
      uint16_t packed_words[600]; // supports up to 2000 samples => 500 words, margin added
      size_t packed_count = pack_samples_to_words(g_logan_sample_buffer, g_logan_sample_index, packed_words, 600);

      // Build header from nibbles
      uint8_t samples_nib = g_logan_samples_nibble;
      uint8_t rate_nib = g_logan_rate_nibble;
      uint8_t type_nib = 0x06;
      uint8_t hdr_csum = (type_nib ^ (samples_nib & 0x0F) ^ (rate_nib & 0x0F)) & 0x0F;
      uint16_t header_word = ((type_nib & 0x0F) << 12) |
                             ((samples_nib & 0x0F) << 8) |
                             ((rate_nib & 0x0F) << 4) |
                             (hdr_csum & 0x0F);

      // Configure custom header with correct payload size in words
      SPIComm::configure_custom_header(header_word, packed_count);

      // Compute checksum across packed payload
      uint16_t payload_checksum = 0;
      for (size_t i = 0; i < packed_count; ++i)
      {
        payload_checksum ^= packed_words[i];
      }

      // Print formatted packet to serial monitor
      printf("LOGAN HEADER: 0x%04X\r\n", header_word);
      printf("LOGAN PAYLOAD (%u words for %u samples):\r\n", (unsigned)packed_count, (unsigned)g_logan_sample_index);
      for (size_t i = 0; i < packed_count; ++i)
      {
        printf("0x%04X%s", packed_words[i], ((i + 1) % 16 == 0 || i + 1 == packed_count) ? "\r\n" : " ");
      }
      printf("LOGAN CHECKSUM: 0x%04X\r\n", payload_checksum);

      // Commit payload and start transfer
      SPIComm::set_logan_payload(packed_words, packed_count);
      SPIComm::start_dummy_samples_transfer();

      // If continuous mode enabled, immediately re-arm sampling for the next block
      if (g_logan_continuous)
      {
        size_t rate_khz = SPIComm::rate_from_nibble(g_logan_rate_nibble);
        if (rate_khz == 0) rate_khz = 1;
        int64_t interval_us = static_cast<int64_t>(1000 / rate_khz);
        if (interval_us <= 0) interval_us = 1;

        cancel_repeating_timer(&g_logan_timer);
        g_logan_sample_index = 0;
        g_logan_sampling_active = true;
        add_repeating_timer_us(-interval_us, logan_timer_callback, nullptr, &g_logan_timer);
        printf("LOGAN RE-ARM: rate_khz=%u interval_us=%lld\r\n",
               (unsigned)rate_khz,
               (long long)interval_us);
      }
    }
    else
    {
      // Defer packaging until TX bulk channel is available
      printf("LOGAN TX busy - packaging deferred\r\n");
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
  gpio_set_function(pin, GPIO_FUNC_PWM);

  float divider = 4.0f;

  uint slice_num = pwm_gpio_to_slice_num(pin);
  uint channel   = pwm_gpio_to_channel(pin);

  uint32_t sys_clk = clock_get_hz(clk_sys);
  uint32_t wrap = (uint32_t)((sys_clk / (divider * PWM_FREQ_HZ)) - 1);

  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel, wrap / 2);
  pwm_set_enabled(slice_num, true);
}

// EOF
