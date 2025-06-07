#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <array>

constexpr size_t PACKET_SIZE = 4;

enum class PacketType : uint8_t
{
  MacroKey = 0x01,
  EncoderRotate = 0x02,
  EncoderSwitch = 0x03,
};

enum class MacroKeyAction : uint8_t
{
  Key1 = 0x01,
  Key2 = 0x02,
  Key3 = 0x03,
  Key4 = 0x04,
  Key5 = 0x05,
};

enum class EncoderRotationAction : uint8_t
{
  CW = 0x01,
  CCW = 0x02,
};

enum class SwitchValue : uint8_t
{
  Pressed = 0x01,
  Released = 0x00,
};

struct Packet
{
  PacketType type;
  uint8_t action;
  uint8_t value;
  uint8_t checksum;

  Packet() = default;

  Packet(PacketType t, uint8_t a, uint8_t v)
      : type(t), action(a), value(v), checksum(compute_checksum(t, a, v)) {}

  static Packet macro_key(MacroKeyAction key, SwitchValue state)
  {
    return Packet(PacketType::MacroKey, static_cast<uint8_t>(key), static_cast<uint8_t>(state));
  }

  static Packet encoder_rotate(EncoderRotationAction direction, uint8_t steps = 1)
  {
    return Packet(PacketType::EncoderRotate, static_cast<uint8_t>(direction), steps);
  }

  static Packet encoder_switch(SwitchValue state)
  {
    return Packet(PacketType::EncoderSwitch, 0x01, static_cast<uint8_t>(state));
  }

  std::array<uint8_t, PACKET_SIZE> to_bytes() const
  {
    return {static_cast<uint8_t>(type), action, value, checksum};
  }

  void from_bytes(const std::array<uint8_t, PACKET_SIZE> &data)
  {
    type = static_cast<PacketType>(data[0]);
    action = data[1];
    value = data[2];
    checksum = data[3];
  }

  bool is_valid() const
  {
    return checksum == compute_checksum(type, action, value);
  }

  uint16_t to_spi_word() const
  {
    auto bytes = to_bytes();
    uint8_t high = ((bytes[0] & 0x0F) << 4) | (bytes[1] & 0x0F);
    uint8_t low = ((bytes[2] & 0x0F) << 4) | (bytes[3] & 0x0F);
    return (static_cast<uint16_t>(high) << 8) | low;
  }

private:
  static uint8_t compute_checksum(PacketType t, uint8_t a, uint8_t v)
  {
    return static_cast<uint8_t>(t) ^ a ^ v;
  }
};

#endif

// EOF
