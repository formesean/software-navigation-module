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
SPIComm spiComm;

volatile bool led_blink_requested = false;
volatile int32_t encoder_position = 0;
volatile uint8_t encoder_last_state = 0;

const uint32_t ENCODER_DEBOUNCE_US = 2000;
const uint32_t ENCODER_SW_DEBOUNCE_US = 20000;
const uint32_t MACRO_KEY_DEBOUNCE_US = 50000;
const int8_t encoder_transition_table[16] = {
    0, 1, -1, 0, -1, 0, 0, 1,
    1, 0, 0, -1, 0, -1, 1, 0};

absolute_time_t last_encoder_event;
absolute_time_t last_encoder_button_event;

// Function Prototypes
void wait_for_usb_connect();
void setup_pins();
void request_led_blink();
void encoder_shared_irq_handler();

int main()
{
  wait_for_usb_connect();
  setup_pins();

  spiComm.init_slave();
  // spiComm.set_message("Hello World!");

  encoder_last_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DT);

  irq_set_exclusive_handler(IO_IRQ_BANK0, encoder_shared_irq_handler);
  irq_set_enabled(IO_IRQ_BANK0, true);

  gpio_set_irq_enabled(ENCODER_CLK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENCODER_DT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENCODER_SW, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

  printf("Ready to scan keypad!\n");
  fflush(stdout);

  static bool prev_state[5] = {true, true, true, true, true};
  static absolute_time_t last_debounce_time[5] = {0};

  while (true)
  {
    absolute_time_t now = get_absolute_time();

    for (int i = 0; i < 5; i++)
    {
      bool current_state = gpio_get(MACRO_KEYS[i]);
      if (current_state != prev_state[i] && absolute_time_diff_us(last_debounce_time[i], now) > MACRO_KEY_DEBOUNCE_US)
      {
        last_debounce_time[i] = now;
        prev_state[i] = current_state;

        if (!current_state)
        {
          Packet pkt = spiComm.create_packet(
              PacketType::MacroKey,
              static_cast<MacroKeyAction>(i + 1),
              SwitchValue::Pressed);

          ringBuffer.push(pkt);
          spiComm.print_packet(pkt);
        }
        else
        {
          Packet pkt = spiComm.create_packet(
              PacketType::MacroKey,
              static_cast<MacroKeyAction>(i + 1),
              SwitchValue::Released);

          ringBuffer.push(pkt);
          spiComm.print_packet(pkt);
        }

        request_led_blink();
        fflush(stdout);
      }
    }

    spiComm.handle(ringBuffer);

    if (led_blink_requested)
    {
      led_blink_requested = false;
      gpio_put(LED_PIN, 1);
      sleep_ms(50);
      gpio_put(LED_PIN, 0);
    }

    sleep_us(100);
    tight_loop_contents();
  }

  return 0;
}

void wait_for_usb_connect()
{
  stdio_init_all();

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

void encoder_shared_irq_handler()
{
  absolute_time_t now = get_absolute_time();

  // Handle encoder rotation (both CLK and DT edges)
  if (gpio_get_irq_event_mask(ENCODER_CLK) || gpio_get_irq_event_mask(ENCODER_DT))
  {
    if (absolute_time_diff_us(last_encoder_event, now) > ENCODER_DEBOUNCE_US)
    {
      last_encoder_event = now;
      uint32_t status = save_and_disable_interrupts();

      uint8_t current_state = (gpio_get(ENCODER_DT) << 1) | gpio_get(ENCODER_CLK);
      uint8_t index = (encoder_last_state << 2) | current_state;
      int8_t delta = encoder_transition_table[index & 0x0F];

      encoder_position = encoder_position + delta;
      encoder_last_state = current_state;
      restore_interrupts(status);

      if (delta != 0)
      {
        // printf("Encoder: %d (%s)\n", encoder_position, delta > 0 ? "CW" : "CCW");

        Packet pkt = spiComm.create_packet(
            PacketType::EncoderRotate,
            delta > 0 ? EncoderRotationAction::CW : EncoderRotationAction::CCW,
            static_cast<uint8_t>(abs(delta)));

        ringBuffer.push(pkt);
        spiComm.print_packet(pkt);
        request_led_blink();
        fflush(stdout);
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
      bool pressed = !gpio_get(ENCODER_SW);

      // printf("Encoder Switch: %s\n", pressed ? "Pressed" : "Released");

      Packet pkt = spiComm.create_packet(
          PacketType::EncoderSwitch,
          pressed ? SwitchValue::Pressed : SwitchValue::Released);

      ringBuffer.push(pkt);
      spiComm.print_packet(pkt);
      request_led_blink();
      fflush(stdout);
    }
  }
}

// EOF
