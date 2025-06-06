#include <stdio.h>
#include <string.h>
#include <cstdint>

#include "spi_comm.hpp"

uint8_t SPIComm::tx_buffer[SPIComm::MAX_MSG_SIZE] = {0};
size_t SPIComm::tx_length = 0;

uint8_t SPIComm::compute_checksum(uint8_t type, uint8_t action, uint8_t value)
{
  return type ^ action ^ value;
}

void SPIComm::init_slave()
{
  spi_init(spi1, SPI_BAUD);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  spi_set_slave(spi1, true);
}

void SPIComm::handle(RingBuffer &ring)
{
  if (spi_is_readable(spi1) || spi_is_writable(spi1))
  {
    uint8_t rx_buf[PACKET_SIZE] = {0};
    uint8_t tx_buf[PACKET_SIZE] = {0};

    Packet response_packet;
    if (ring.pop(response_packet))
    {
      auto packet_array = response_packet.to_array();
      memcpy(tx_buf, packet_array.data(), PACKET_SIZE);
    }

    int bytes_transferred = spi_write_read_blocking(spi1, tx_buf, rx_buf, PACKET_SIZE);

    if (bytes_transferred == PACKET_SIZE)
    {
      bool has_data = false;
      for (int i = 0; i < PACKET_SIZE; i++)
      {
        if (rx_buf[i] != 0)
        {
          has_data = true;
          break;
        }
      }

      if (has_data)
      {
        Packet received_packet;
        std::array<uint8_t, PACKET_SIZE> rx_array;
        memcpy(rx_array.data(), rx_buf, PACKET_SIZE);
        received_packet.from_array(rx_array);

        if (verify_packet(received_packet))
        {
          printf("Received valid packet from master\n");
          print_packet(received_packet);
        }
      }
    }
  }
}

void SPIComm::print_packet(const Packet &pkt)
{
  bool is_valid = verify_packet(pkt);
  printf("Packet - Type: 0x%02X, Action: 0x%02X, Value: 0x%02X, Checksum: 0x%02X, Valid: %s\n",
         static_cast<uint8_t>(pkt.get_type()),
         pkt.get_action(),
         pkt.get_value(),
         pkt.get_checksum(),
         is_valid ? "true" : "false");
  fflush(stdout);
}

bool SPIComm::verify_packet(const Packet &pkt)
{
  return pkt.get_checksum() == compute_checksum(
                                   static_cast<uint8_t>(pkt.get_type()),
                                   pkt.get_action(),
                                   pkt.get_value());
}

Packet SPIComm::create_packet(PacketType type, MacroKeyAction action, SwitchValue value)
{
  Packet pkt;
  pkt.set_type(type);
  pkt.set_action(static_cast<uint8_t>(action));
  pkt.set_value(static_cast<uint8_t>(value));
  pkt.compute_and_set_checksum();
  return pkt;
}

Packet SPIComm::create_packet(PacketType type, EncoderRotationAction action, uint8_t value)
{
  Packet pkt;
  pkt.set_type(type);
  pkt.set_action(static_cast<uint8_t>(action));
  pkt.set_value(value);
  pkt.compute_and_set_checksum();
  return pkt;
}

Packet SPIComm::create_packet(PacketType type, SwitchValue value)
{
  Packet pkt;
  pkt.set_type(type);
  pkt.set_action(static_cast<uint8_t>(0x01));
  pkt.set_value(static_cast<uint8_t>(value));
  pkt.compute_and_set_checksum();
  return pkt;
}

void SPIComm::set_message(const char *msg)
{
  size_t len = strlen(msg);
  if (len > SPIComm::MAX_MSG_SIZE)
    len = SPIComm::MAX_MSG_SIZE;
  memcpy(tx_buffer, msg, len);
  tx_length = len;
}

// EOF
