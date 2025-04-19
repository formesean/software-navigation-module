#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>

#include <iostream>
#include <cstdint>
#include <stdio.h>
#include <string.h>

#define COL1 9
#define COL2 8
#define COL3 7
#define ROW1 6
#define ROW2 5

#define SW_PIN 4
#define DT_PIN 3
#define CLK_PIN 2

const uint8_t colPins[3] = {COL1, COL2, COL3};
const uint8_t rowPins[2] = {ROW1, ROW2};
volatile int val = 0;
volatile bool prev_clk = 1;

void setup_pins()
{
  for (uint8_t col : colPins)
  {
    gpio_init(col);
    gpio_set_dir(col, GPIO_OUT);
    gpio_put(col, 1); // Idle HIGH
  }

  for (uint8_t row : rowPins)
  {
    gpio_init(row);
    gpio_set_dir(row, GPIO_IN);
    gpio_pull_up(row); // Pull-up so unpressed reads HIGH
  }

  // GPIO Setup for Encoder
  gpio_init(SW_PIN);
  gpio_set_dir(SW_PIN, GPIO_IN);
  gpio_pull_up(SW_PIN);

  gpio_init(DT_PIN);
  gpio_set_dir(DT_PIN, GPIO_IN);
  gpio_disable_pulls(DT_PIN);

  gpio_init(CLK_PIN);
  gpio_set_dir(CLK_PIN, GPIO_IN);
  gpio_disable_pulls(CLK_PIN);
}

void wait_for_usb_connect()
{
  stdio_init_all();

  // Timeout after 5 seconds
  absolute_time_t timeout = make_timeout_time_ms(5000);
  while (!stdio_usb_connected() && !time_reached(timeout))
    sleep_ms(10);

  sleep_ms(100);
}

void encoder_callback(uint gpio, uint32_t events)
{
  uint8_t clk_state = gpio_get(CLK_PIN);
  uint8_t dt_state = gpio_get(DT_PIN);
  uint8_t sw_state = gpio_get(SW_PIN);

  if (prev_clk == 1 && clk_state == 0)
  {
    if (dt_state == 0)
    {
      val = (val - 1 + 6) % 6; // CCW
      printf("CCW %d\r\n", val);
    }
    else
    {
      val = (val + 1) % 6; // CW
      printf("CW %d\r\n", val);
    }
  }

  prev_clk = clk_state;

  if (gpio == SW_PIN && sw_state == 0)
    printf("SW pressed!\r\n");
}

int main()
{
  wait_for_usb_connect();
  setup_pins();

  // Enable interrupts for SW, DT, and CLK pins with the proper callback function
  gpio_set_irq_enabled_with_callback(SW_PIN, GPIO_IRQ_EDGE_FALL, true, encoder_callback);
  gpio_set_irq_enabled_with_callback(CLK_PIN, GPIO_IRQ_EDGE_FALL, true, encoder_callback);
  gpio_set_irq_enabled_with_callback(DT_PIN, GPIO_IRQ_EDGE_FALL, true, encoder_callback);

  printf("Ready to scan keypad!\n");
  fflush(stdout);

  while (true)
  {
    for (int colIdx = 0; colIdx < 3; colIdx++)
    {
      gpio_put(colPins[colIdx], 0); // Pull column LOW

      for (int rowIdx = 0; rowIdx < 2; rowIdx++)
      {
        if (gpio_get(rowPins[rowIdx]) == 0)
        { // Active LOW
          uint8_t keyId = rowIdx * 3 + colIdx;

          printf("Key Pressed: ID=%d, HEX=0x%02X, BIN=0b", keyId, keyId);
          for (int b = 7; b >= 0; b--)
          {
            printf("%d", (keyId >> b) & 1);
          }
          printf("\n");
          fflush(stdout);

          sleep_ms(300); // Debounce
        }
      }

      gpio_put(colPins[colIdx], 1); // Release column
    }
  }

  return 0;
}
