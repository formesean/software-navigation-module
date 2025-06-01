#ifndef SPI_COMM_HPP
#define SPI_COMM_HPP

#include <pico/stdlib.h>
#include <hardware/spi.h>

#include "ring_buffer.hpp"
#include "packet.hpp"

class SPIComm
{
private:
  static constexpr uint32_t SPI_BAUD = 1000000;
  static constexpr uint8_t PIN_SCK = 10;
  static constexpr uint8_t PIN_MISO = 11;
  static constexpr uint8_t PIN_MOSI = 12;
  static constexpr uint8_t PIN_CS = 13;

  static uint8_t compute_checksum(uint8_t type, uint8_t action, uint8_t value);

public:
  static constexpr size_t MAX_MSG_SIZE = 64;
  static uint8_t tx_buffer[MAX_MSG_SIZE];
  static size_t tx_length;

  static void init_slave();
  static void handle(RingBuffer &ring);
  static void print_packet(const Packet &pkt);
  static bool verify_packet(const Packet &pkt);

  // Overloads for creating type-safe packets
  static Packet create_packet(PacketType type, MacroKeyAction action, SwitchValue value);
  static Packet create_packet(PacketType type, EncoderRotationAction action, uint8_t value);
  static Packet create_packet(PacketType type, SwitchValue value);

  static void set_message(const char *msg);
};

#endif

// EOF
