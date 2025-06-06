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
  spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  spi_set_slave(spi1, true);

  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
}

void SPIComm::handle(RingBuffer &ring)
{
  uint8_t rx_buf[PACKET_SIZE] = {0};
  uint8_t tx_buf[PACKET_SIZE] = {0};

  Packet packet_to_send;

  if (ring.pop(packet_to_send))
  {
    auto packet_array = packet_to_send.to_array();
    memcpy(tx_buf, packet_array.data(), PACKET_SIZE);

    uint8_t high_byte = ((tx_buf[0] & 0x0F) << 4) | (tx_buf[1] & 0x0F);
    uint8_t low_byte = ((tx_buf[2] & 0x0F) << 4) | (tx_buf[3] & 0x0F);

    uint16_t tx_data = (static_cast<uint16_t>(high_byte) << 8) | low_byte;
    uint16_t rx_dummy = 0;

    int result = spi_write16_read16_blocking(spi1, &tx_data, &rx_dummy, 1);

    if (result == 1)
    {
      printf("Packet Sent: 0x%04X\n", tx_data);
      printf("Original packet data: [0x%02X, 0x%02X, 0x%02X, 0x%02X]\n",
             tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
      fflush(stdout);
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
