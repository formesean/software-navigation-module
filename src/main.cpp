#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
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

// State Variables
RingBuffer ringBuffer;

volatile bool led_blink_requested = false;
volatile int32_t encoder_position = 0;
volatile uint8_t encoder_last_state = 0;

// Timing constants
const uint32_t ENCODER_DEBOUNCE_US = 500;
const uint32_t ENCODER_SW_DEBOUNCE_US = 10000;
const uint32_t MACRO_KEY_DEBOUNCE_US = 10000;

// Encoder state transition table
const int8_t encoder_transition_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1,
    -1, 0, 0, 1, 0, 1, -1, 0};

absolute_time_t last_encoder_event;
absolute_time_t last_encoder_button_event;

bool prev_state[5] = {true, true, true, true, true};
absolute_time_t last_debounce_time[5] = {0};

// Function Prototypes
void wait_for_usb_connect();
void setup_pins();
void request_led_blink();
void shared_irq_handler();
void process_buffered_events();

int main()
{
  stdio_init_all();
  wait_for_usb_connect();
  setup_pins();
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

    __wfi();
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

void request_led_blink()
{
  led_blink_requested = true;
}

void shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();

  // Handle macro keys
  for (int i = 0; i < 5; ++i)
  {
    uint key_pin = MACRO_KEYS[i];

    if (gpio_get_irq_event_mask(key_pin) & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE))
    {
      if (absolute_time_diff_us(last_debounce_time[i], now) > MACRO_KEY_DEBOUNCE_US)
      {
        last_debounce_time[i] = now;
        gpio_acknowledge_irq(key_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);

        bool current_pressed = !gpio_get(key_pin);

        if (current_pressed != prev_state[i])
        {
          prev_state[i] = current_pressed;

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
    if (absolute_time_diff_us(last_encoder_event, now) > ENCODER_DEBOUNCE_US)
    {
      last_encoder_event = now;

      uint32_t status = save_and_disable_interrupts();
      uint8_t current_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);
      uint8_t transition_index = (encoder_last_state << 2) | current_state;
      int8_t delta = encoder_transition_table[transition_index & 0x0F];

      encoder_position += delta;
      encoder_last_state = current_state;

      restore_interrupts(status);

      if (delta != 0)
      {
        bool clockwise = delta > 0;
        uint8_t steps = static_cast<uint8_t>(abs(delta));

        uint16_t event_data = SPIComm::create_encoder_rotate_event(clockwise, steps);
        if (!SPIComm::queue_packet(event_data))
        {
          ringBuffer.push(event_data);
        }
        request_led_blink();
      }

      gpio_acknowledge_irq(ENCODER_CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
      gpio_acknowledge_irq(ENCODER_DT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    }
  }

  // Handle encoder switch
  if (gpio_get_irq_event_mask(ENCODER_SW) & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE))
  {
    if (absolute_time_diff_us(last_encoder_button_event, now) > ENCODER_SW_DEBOUNCE_US)
    {
      last_encoder_button_event = now;
      gpio_acknowledge_irq(ENCODER_SW, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);

      bool current_pressed = !gpio_get(ENCODER_SW);

      uint16_t event_data = SPIComm::create_encoder_switch_event(current_pressed);
      if (!SPIComm::queue_packet(event_data))
      {
        ringBuffer.push(event_data);
      }
      request_led_blink();
    }
  }
}

void process_buffered_events()
{
  uint16_t event_data;

  while (ringBuffer.pop(event_data))
  {
    if (SPIComm::send_packet(event_data))
    {
      continue;
    }
    else
    {
      ringBuffer.push(event_data);
      break;
    }
  }
}

// EOF
