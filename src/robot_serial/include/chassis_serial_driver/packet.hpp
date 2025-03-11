#ifndef CHASSIS_SERIAL_DRIVER__PACKET_HPP_
#define CHASSIS_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace chassis_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t Rxdata;  // 0-red 1-blue

  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  double position_x;
  double position_y;
  double position_z;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}


}  // namespace chassis_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
