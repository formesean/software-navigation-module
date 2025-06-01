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
  if (spi_is_readable(spi1))
  {
    uint8_t rx_buf[PACKET_SIZE] = {0};
    uint8_t tx_buf[PACKET_SIZE] = {0};

    // Read packet from SPI
    spi_read_blocking(spi1, 0x00, rx_buf, PACKET_SIZE);

    Packet received_packet;
    memcpy(&received_packet, rx_buf, PACKET_SIZE);

    // Verify and push if valid
    if (verify_packet(received_packet))
    {
      print_packet(received_packet);
      ring.push(received_packet);
    }

    // Send dynamic message instead of packet
    if (tx_length > 0)
    {
      spi_write_blocking(spi1, tx_buffer, tx_length);
      tx_length = 0;
    }
    else
    {
      Packet response_packet;
      if (!ring.pop(response_packet))
        memset(&response_packet, 0, sizeof(Packet));

      uint8_t tx_buf[PACKET_SIZE] = {0};
      memcpy(tx_buf, &response_packet, PACKET_SIZE);
      spi_write_blocking(spi1, tx_buf, PACKET_SIZE);
    }
  }
}

void SPIComm::print_packet(const Packet &pkt)
{
  bool is_valid = verify_packet(pkt);
  printf("Packet - Type: 0x%02X, Action: 0x%02X, Value: 0x%02X, Checksum: 0x%02X, Verified: %s\n",
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
  pkt.set_checksum(compute_checksum(static_cast<uint8_t>(type), static_cast<uint8_t>(action), static_cast<uint8_t>(value)));
  return pkt;
}

Packet SPIComm::create_packet(PacketType type, EncoderRotationAction action, uint8_t value)
{
  Packet pkt;
  pkt.set_type(type);
  pkt.set_action(static_cast<uint8_t>(action));
  pkt.set_value(value);
  pkt.set_checksum(compute_checksum(static_cast<uint8_t>(type), static_cast<uint8_t>(action), value));
  return pkt;
}

Packet SPIComm::create_packet(PacketType type, SwitchValue value)
{
  Packet pkt;
  pkt.set_type(type);
  pkt.set_action(static_cast<uint8_t>(0x01));
  pkt.set_value(static_cast<uint8_t>(value));
  pkt.set_checksum(compute_checksum(static_cast<uint8_t>(type), static_cast<uint8_t>(0x01), static_cast<uint8_t>(value)));
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
