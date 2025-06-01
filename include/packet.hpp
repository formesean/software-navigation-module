#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <array>
#include <cstring>
#include <iostream>

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

class Packet
{
private:
  PacketType _type;
  uint8_t _action;
  uint8_t _value;
  uint8_t _checksum;

  static uint8_t compute_checksum(PacketType type, uint8_t action, uint8_t value)
  {
    return static_cast<uint8_t>(type) ^ (action) ^ (value);
  }

public:
  Packet() = default;

  // Getters
  PacketType get_type() const { return _type; }
  uint8_t get_action() const { return _action; }
  uint8_t get_value() const { return _value; }
  uint8_t get_checksum() const { return _checksum; }

  // Setters
  void set_type(PacketType type) { _type = type; }
  void set_action(uint8_t action) { _action = action; }
  void set_value(uint8_t value) { _value = value; }
  void set_checksum(uint8_t checksum) { _checksum = checksum; }

  // Calculate and assign checksum
  void compute_and_set_checksum()
  {
    _checksum = compute_checksum(_type, _action, _value);
  }

  // Convert to byte array for SPI transmission
  std::array<uint8_t, PACKET_SIZE> to_array() const
  {
    return {
        static_cast<uint8_t>(_type),
        _action,
        _value,
        _checksum};
  }

  // Load from byte array (from SPI buffer)
  void from_array(const std::array<uint8_t, PACKET_SIZE> &data)
  {
    _type = static_cast<PacketType>(data[0]);
    _action = data[1];
    _value = data[2];
    _checksum = data[3];
  }

  // Validate if checksum matches the type, action, value
  bool validate_checksum() const
  {
    return _checksum == compute_checksum(_type, _action, _value);
  }
};

#endif

// EOF
