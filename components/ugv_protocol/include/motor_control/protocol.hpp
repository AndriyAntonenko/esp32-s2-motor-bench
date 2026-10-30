#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <optional>
#include <array>

// single frame:
// -----------------           <---- header starts
// SOF      | 1 byte
// LEN.     | 1 byte
// SEQ      | 1 byte
// TYPE     | 1 byte
// -----------------           <---- header ends
// PAYLOAD  | up to 64 bytes
// -----------------
// CRC      | 2 bytes
// -----------------

namespace motor_control::protocol
{
inline constexpr std::uint8_t kProtocolVersion = 1;
inline constexpr uint8_t kSof = 0xA5;
inline constexpr size_t kMaxPayload = 64;
inline constexpr size_t kHeaderSize = 4;
inline constexpr size_t kCRCSize = 2;
inline constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayload + kCRCSize;

enum class MessageType : std::uint8_t
{
  Hello = 0x01,
  Set = 0x02,
  Enable = 0x03,
  Disable = 0x04,
  Info = 0x81,
  Status = 0x82
};

[[nodiscard]]
uint16_t crc16_update(std::uint16_t crc, std::uint8_t byte);

[[nodiscard]]
uint16_t crc16(std::span<const std::uint8_t> data);

[[nodiscard]]
std::size_t encode_frame(
  MessageType type, std::uint8_t seq, std::span<const std::uint8_t> payload,
  std::span<std::uint8_t> out);

enum class Direction: int8_t
{
  Forward = 1,
  Reverse = -1
};

constexpr const char * to_string(Direction d)
{
  return d == Direction::Forward ? "FWD" : "REV";
}

constexpr const uint16_t kMaxPower = 10000;

struct MotorCommand
{
  uint16_t power{0};
  Direction direction{Direction::Forward};
};

inline constexpr std::size_t kSetEntrySize = 3;

[[nodiscard]]
std::size_t encode_set(
  std::uint8_t seq, std::span<const MotorCommand> motors,
  std::span<std::uint8_t> out);

struct Frame
{
  MessageType type;
  std::uint8_t seq;
  std::uint8_t len;
  std::array<std::uint8_t, kMaxPayload> payload;
};

class FrameParser {
public:
  enum class Result { Incomplete, FrameReady, CrcError, LengthError };
  Result push(std::uint8_t byte);
  const Frame & frame() {return frame_;}
  void reset();

private:
  enum class State {WaitSof, Len, Seq, Type, Payload, CrcLo, CrcHi};
  State state_{State::WaitSof};
  Frame frame_{};
  std::size_t index_{0};
  std::uint16_t crc_{0};
  std::uint16_t rx_crc_{0};
};

enum class DeviceState : std::uint8_t { Disabled = 0, Enabled = 1, Failsafe = 2 };

namespace fault
{
inline constexpr std::uint8_t kWatchdog = 1u << 0;
inline constexpr std::uint8_t kBadSet = 1u << 1;
inline constexpr std::uint8_t kLowBattery = 1u << 2;
inline constexpr std::uint8_t kNotRosSource = 1u << 3;
inline constexpr std::uint8_t kClamped = 1u << 4;
}  // namespace fault

inline constexpr size_t kInfoSize = 7;
inline constexpr size_t kStatusHeaderSize = 8;
inline constexpr size_t kMaxStatusMotors = (kMaxPayload - kStatusHeaderSize) / kSetEntrySize;

struct Info
{
  uint8_t protocol_version{0};
  uint8_t motor_count{0};
  uint16_t command_limit{0};
  uint16_t watchdog_ms{0};
  uint8_t fw_version{0};
};

struct Status
{
  DeviceState state{DeviceState::Disabled};
  uint8_t fault_flags{0};
  uint8_t ack_seq{0};
  uint16_t rx_crc_errors{0};
  uint16_t battery_mv{0};
  uint8_t motor_count{0};
  std::array<MotorCommand, kMaxStatusMotors> applied{};
};

[[nodiscard]] std::optional<Info> decode_info(std::span<const std::uint8_t> payload);
[[nodiscard]] std::optional<Status> decode_status(std::span<const std::uint8_t> payload);
}
