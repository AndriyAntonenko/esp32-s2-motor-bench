#include "motor_control/protocol.hpp"

#include <cstdint>
#include <cstddef>
#include <span>
#include <algorithm>
#include <array>
#include <optional>

namespace motor_control::protocol
{
namespace
{
std::uint16_t read_u16_le(std::span<const std::uint8_t> p, std::size_t offset)
{
  return static_cast<std::uint16_t>(p[offset] | (p[offset + 1] << 8));
}

MotorCommand read_motor(std::span<const std::uint8_t> p, std::size_t offset)
{
  // u8 -> i8 first (0xFF -> -1), then to the enum
  const auto dir = static_cast<std::int8_t>(p[offset + 2]);
  return MotorCommand{read_u16_le(p, offset), static_cast<Direction>(dir)};
}

bool is_valid(const MotorCommand & m)
{
  const bool direction_ok =
    m.direction == Direction::Forward || m.direction == Direction::Reverse;
  return m.power <= kMaxPower && direction_ok;
}
}

std::uint16_t crc16_update(std::uint16_t crc, std::uint8_t byte)
{
  crc ^= static_cast<std::uint16_t>(byte) << 8;
  for (std::size_t j = 0; j < 8; j++) {
    if (crc & 0x8000) {
      crc = static_cast<std::uint16_t>(crc << 1) ^ 0x1021;
    } else {
      crc = static_cast<std::uint16_t>(crc << 1);
    }
  }

  return crc;
}

std::uint16_t crc16(std::span<const std::uint8_t> data)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t byte : data) {
    crc = crc16_update(crc, byte);
  }
  return crc;
}

std::size_t encode_frame(
  MessageType type, std::uint8_t seq, std::span<const std::uint8_t> payload,
  std::span<std::uint8_t> out)
{
  const std::size_t frame_size = kHeaderSize + payload.size() + kCRCSize;

  if (payload.size() > kMaxPayload || out.size() < frame_size) {
    return 0;
  }

  // header
  out[0] = kSof;
  out[1] = static_cast<std::uint8_t>(payload.size());
  out[2] = seq;
  out[3] = static_cast<std::uint8_t>(type);

  // payload
  std::ranges::copy(payload, out.begin() + kHeaderSize);

  const std::uint16_t crc = crc16(out.subspan(1, frame_size - kCRCSize - 1));
  out[frame_size - 2] = static_cast<std::uint8_t>(crc & 0xFF);
  out[frame_size - 1] = static_cast<std::uint8_t>(crc >> 8);

  return frame_size;
}

std::size_t encode_set(
  std::uint8_t seq, std::span<const MotorCommand> motors,
  std::span<std::uint8_t> out)
{
  if (motors.size() * kSetEntrySize > kMaxPayload) {
    return 0;
  }

  if (std::ranges::any_of(motors, [](const MotorCommand & m ) {return !is_valid(m);})) {
    return 0;
  }

  std::array<std::uint8_t, kMaxPayload> payload{};
  std::size_t pos = 0;
  for (auto cmd : motors) {
    // little endian
    payload[pos++] = static_cast<std::uint8_t>(cmd.power & 0xFF);
    payload[pos++] = static_cast<std::uint8_t>(cmd.power >> 8);
    payload[pos++] = static_cast<std::uint8_t>(cmd.direction);
  }

  return encode_frame(MessageType::Set, seq, std::span(payload).first(pos), out);
}

FrameParser::Result FrameParser::push(std::uint8_t byte)
{
  switch (state_) {
    case State::WaitSof:
      if (byte == kSof) {
        crc_ = 0xFFFF;
        state_ = State::Len;
      }
      return Result::Incomplete;

    case State::Len:
      if (byte > kMaxPayload) {
        state_ = State::WaitSof;
        return Result::LengthError;
      }

      frame_.len = byte;
      state_ = State::Seq;
      crc_ = crc16_update(crc_, byte);
      return Result::Incomplete;

    case State::Seq:
      frame_.seq = byte;
      state_ = State::Type;
      crc_ = crc16_update(crc_, byte);
      return Result::Incomplete;

    case State::Type:
      frame_.type = static_cast<MessageType>(byte);
      state_ = frame_.len == 0 ? State::CrcLo : State::Payload;
      crc_ = crc16_update(crc_, byte);
      index_ = 0;
      return Result::Incomplete;

    case State::Payload:
      frame_.payload[index_++] = byte;
      crc_ = crc16_update(crc_, byte);

      if (index_ == frame_.len) {
        state_ = State::CrcLo;
      }
      return Result::Incomplete;

    case State::CrcLo:
      rx_crc_ = static_cast<uint16_t>(byte);
      state_ = State::CrcHi;
      return Result::Incomplete;

    case State::CrcHi:
      rx_crc_ |= static_cast<uint16_t>(byte << 8);
      state_ = State::WaitSof;
      return rx_crc_ == crc_ ? Result::FrameReady : Result::CrcError;
  }

  state_ = State::WaitSof;
  return Result::Incomplete;
}

void FrameParser::reset()
{
  state_ = State::WaitSof;
  frame_ = Frame{};
  index_ = 0;
  crc_ = 0;
  rx_crc_ = 0;
}

std::optional<Info> decode_info(std::span<const std::uint8_t> payload)
{
  if (payload.size() != kInfoSize) {
    return std::nullopt;
  }

  Info info;
  info.protocol_version = payload[0];
  info.motor_count = payload[1];
  info.command_limit = read_u16_le(payload, 2);
  info.watchdog_ms = read_u16_le(payload, 4);
  info.fw_version = payload[6];
  return info;
}

std::optional<Status> decode_status(std::span<const std::uint8_t> payload)
{
  // Check sizes before reading anything: span::operator[] has no bounds check.
  if (payload.size() < kStatusHeaderSize) {
    return std::nullopt;
  }

  const std::size_t motor_count = payload[7];
  if (motor_count > kMaxStatusMotors) {
    return std::nullopt;
  }
  if (payload.size() != kStatusHeaderSize + motor_count * kSetEntrySize) {
    return std::nullopt;
  }
  if (payload[0] > static_cast<std::uint8_t>(DeviceState::Failsafe)) {
    return std::nullopt;
  }

  Status status;
  status.state = static_cast<DeviceState>(payload[0]);
  status.fault_flags = payload[1];
  status.ack_seq = payload[2];
  status.rx_crc_errors = read_u16_le(payload, 3);
  status.battery_mv = read_u16_le(payload, 5);
  status.motor_count = payload[7];

  for (std::size_t i = 0; i < motor_count; ++i) {
    const MotorCommand cmd = read_motor(payload, kStatusHeaderSize + i * kSetEntrySize);
    if (!is_valid(cmd)) {
      return std::nullopt;
    }
    status.applied[i] = cmd;
  }
  return status;
}
}
