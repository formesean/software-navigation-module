#include <pico/stdlib.h>
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

// Timing constants
const uint32_t ENCODER_DEBOUNCE_US = 2000;
const uint32_t ENCODER_SW_DEBOUNCE_US = 25000;
const uint32_t MACRO_KEY_DEBOUNCE_US = 25000;

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

  printf("Ready to scan keypad!\n");
  fflush(stdout);

  while (true)
  {
    process_buffered_events();

    if (led_blink_requested)
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

void shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();
  uint32_t irq_status = save_and_disable_interrupts();

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
          printf("Macro key %d %s\n", i + 1, current_pressed ? "pressed" : "released");
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
          printf("Encoder rotated %s by %d steps\n", clockwise ? "CW" : "CCW", steps);
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
        printf("Encoder switch %s\n", current_pressed ? "pressed" : "released");
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

  uint16_t rx_snapshot = g_rx_word;
  if (rx_snapshot != 0)
  {
    uint8_t rx_high_byte = (rx_snapshot >> 8) & 0xFF;
    uint8_t rx_low_byte = rx_snapshot & 0xFF;
    bool send_samples = (rx_high_byte >> 4) == 0x06;
    if (rx_high_byte != 0x00 && rx_low_byte != 0x00)
    {
      printf("Received: 0x%04X (bytes: 0x%02X 0x%02X)\n", rx_snapshot, rx_high_byte, rx_low_byte);
      g_rx_word = 0;

      if (send_samples)
      {
        SPIComm::start_dummy_samples_transfer();
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
      printf("Queued buffered event: 0x%04X\n", event_data);
    }
    else
    {
      ringBuffer.push(event_data);
      break;
    }

    sleep_us(500);
  }

  // Dummy samples are now streamed by SPI TX FIFO preloading within SPIComm

  static absolute_time_t last_debug_time = 0;
  if (absolute_time_diff_us(last_debug_time, now) > 5000000)
  {
    last_debug_time = now;
    printf("Buffer utilization: %d%%, SPI status: %d\n",
           ringBuffer.utilization_percent(), SPIComm::get_queue_status());
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
