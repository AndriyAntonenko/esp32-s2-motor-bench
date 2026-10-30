// Motor controller example for the UGV UART protocol v1
// See README.md next to this project and docs/motor_control_node/protocol_spec.md.
//
// Bench: LOLIN S2 mini + L298N + two TT motors.
//   UART1: GPIO17 TX -> Pi GPIO15 (RXD0), GPIO18 RX <- Pi GPIO14 (TXD0), common GND.
//   Logs go to the USB CDC console, the protocol has UART1 to itself.

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control/protocol.hpp"

namespace
{
namespace proto = motor_control::protocol;
using proto::DeviceState;
using proto::Direction;
using proto::MessageType;
using proto::MotorCommand;

constexpr const char * TAG = "motor_ctrl";

// --- hardware -------------------------------------------------------------
constexpr uart_port_t kUart = UART_NUM_1;
constexpr int kUartTxPin = 17;
constexpr int kUartRxPin = 18;
constexpr int kBaud = 115200;
constexpr int kUartRxBuffer = 512;     // bytes; ~44 ms of traffic at 115200

constexpr int kPwmFreqHz = 2000;
constexpr uint32_t kMaxDuty = 200;     // of 255 (8 bit): 9 V supply, 6 V motors
constexpr std::size_t kMotorCount = 2;
// L298N inputs per channel: {forward pin, reverse pin}. Channel index = motor_index in the URDF.
constexpr int kPins[kMotorCount][2] = {
  {3, 5},  // ch0: A, B
  {7, 9},  // ch1: C, D
};

// --- protocol parameters, reported in INFO ----------------------------------
constexpr uint16_t kCommandLimit = proto::kMaxPower;
constexpr uint16_t kWatchdogMs = 300;
constexpr uint8_t kFwVersion = 1;
constexpr int64_t kStatusPeriodUs = 100'000;   // STATUS at 10 Hz
constexpr int64_t kSetLogPeriodUs = 1'000'000; // print applied SET at most once a second
constexpr int64_t kHeartbeatUs = 2'000'000;    // "alive" line: the USB console is silent otherwise

// --- motors -----------------------------------------------------------------
void pwm_init()
{
  ledc_timer_config_t timer{};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;  // S2 has only low-speed mode
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = kPwmFreqHz;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  for (std::size_t m = 0; m < kMotorCount; ++m) {
    for (int i = 0; i < 2; ++i) {
      ledc_channel_config_t ch{};
      ch.gpio_num = kPins[m][i];
      ch.speed_mode = LEDC_LOW_SPEED_MODE;
      ch.channel = static_cast<ledc_channel_t>(m * 2 + i);
      ch.timer_sel = LEDC_TIMER_0;
      ch.duty = 0;
      ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
  }
}

void set_duty(int channel, uint32_t duty)
{
  ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel));
}

// power 0..10000 -> duty 0..kMaxDuty. Forward: PWM on A, B low. Reverse: the other way round.
void drive(std::size_t motor, const MotorCommand & cmd)
{
  const uint32_t duty = static_cast<uint32_t>(cmd.power) * kMaxDuty / proto::kMaxPower;
  const int fwd = static_cast<int>(motor * 2);
  const int rev = fwd + 1;
  if (cmd.direction == Direction::Forward) {
    set_duty(rev, 0);
    set_duty(fwd, duty);
  } else {
    set_duty(fwd, 0);
    set_duty(rev, duty);
  }
}

// --- little-endian helpers --------------------------------------------------
void put_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}

uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

const char * reset_reason_name(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_EXT:      return "external pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply dropped)";
    default:               return "other";
  }
}

const char * state_name(DeviceState s)
{
  switch (s) {
    case DeviceState::Disabled: return "DISABLED";
    case DeviceState::Enabled:  return "ENABLED";
    case DeviceState::Failsafe: return "FAILSAFE";
  }
  return "?";
}

// --- self test: the shared codec must produce the spec's test vectors -------
bool self_test()
{
  constexpr std::array<uint8_t, 9> kCheck{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  if (proto::crc16(kCheck) != 0x29B1) {
    return false;
  }
  constexpr std::array<uint8_t, 6> kHello{0xA5, 0x00, 0x00, 0x01, 0xBD, 0xDC};
  std::array<uint8_t, proto::kMaxFrameSize> out{};
  const std::size_t n = proto::encode_frame(MessageType::Hello, 0, {}, out);
  return n == kHello.size() && std::equal(kHello.begin(), kHello.end(), out.begin());
}

// --- the controller ---------------------------------------------------------
class MotorController
{
public:
  void init()
  {
    pwm_init();  // first: outputs at 0 before anything else can happen

    uart_config_t cfg{};
    cfg.baud_rate = kBaud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    // TX buffer 0: uart_write_bytes copies into the 128-byte HW FIFO; a frame is at most 70.
    ESP_ERROR_CHECK(uart_driver_install(kUart, kUartRxBuffer, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kUart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(kUart, kUartTxPin, kUartRxPin,
      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "ready: %u channels, limit %u, watchdog %u ms, UART1 TX=%d RX=%d @ %d",
      static_cast<unsigned>(kMotorCount), kCommandLimit, kWatchdogMs,
      kUartTxPin, kUartRxPin, kBaud);
  }

  [[noreturn]] void run()
  {
    std::array<uint8_t, 128> buf{};
    while (true) {
      // Waits up to 10 ms: bounds the latency of the watchdog and STATUS checks below.
      const int n = uart_read_bytes(kUart, buf.data(), buf.size(), pdMS_TO_TICKS(10));
      for (int i = 0; i < n; ++i) {
        switch (parser_.push(buf[i])) {
          case proto::FrameParser::Result::FrameReady:
            ++rx_frames_;
            handle(parser_.frame());
            break;
          case proto::FrameParser::Result::CrcError:
            if (rx_crc_errors_ < 0xFFFF) {
              ++rx_crc_errors_;
            }
            break;
          default:
            break;
        }
      }

      const int64_t now = esp_timer_get_time();
      if (state_ == DeviceState::Enabled &&
        now - last_valid_set_us_ > int64_t{kWatchdogMs} * 1000)
      {
        faults_ |= proto::fault::kWatchdog;
        change_state(DeviceState::Failsafe, "watchdog: no valid SET");
        send_status();
      }
      if (now >= next_status_us_) {
        send_status();
      }
      if (now >= next_heartbeat_us_) {
        next_heartbeat_us_ = now + kHeartbeatUs;
        log_heartbeat(now);
      }
    }
  }

private:
  // Bench has a single control source. A real vehicle returns false while ELRS/WEB drive.
  static bool ros_is_active_source() {return true;}

  void handle(const proto::Frame & f)
  {
    switch (f.type) {
      case MessageType::Hello:
        ack_seq_ = f.seq;
        ESP_LOGI(TAG, "HELLO -> INFO");
        send_info();
        break;

      case MessageType::Enable:
        ack_seq_ = f.seq;
        if (!ros_is_active_source()) {
          faults_ |= proto::fault::kNotRosSource;
          ESP_LOGW(TAG, "ENABLE refused: ROS is not the active source");
        } else {
          faults_ = 0;                                  // a successful ENABLE clears the flags
          last_valid_set_us_ = esp_timer_get_time();    // watchdog starts now
          change_state(DeviceState::Enabled, "ENABLE");
        }
        send_status();                                  // the Pi waits for this ack
        break;

      case MessageType::Disable:
        ack_seq_ = f.seq;
        change_state(DeviceState::Disabled, "DISABLE");
        send_status();
        break;

      case MessageType::Set:
        handle_set(f);
        break;

      default:
        break;  // unknown type, or one we send ourselves: ignore
    }
  }

  void handle_set(const proto::Frame & f)
  {
    if (state_ != DeviceState::Enabled) {
      return;  // DISABLED / FAILSAFE: ignore, leaving FAILSAFE needs an explicit ENABLE
    }

    // Validate the whole frame first: SET is applied completely or not at all.
    if (f.len % proto::kSetEntrySize != 0 || f.len / proto::kSetEntrySize != kMotorCount) {
      faults_ |= proto::fault::kBadSet;
      ESP_LOGW(TAG, "SET rejected: LEN %u", f.len);
      return;  // watchdog NOT restarted
    }

    std::array<MotorCommand, kMotorCount> cmd{};
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const uint8_t * e = &f.payload[i * proto::kSetEntrySize];
      uint16_t power = get_u16(e);
      const auto dir = static_cast<int8_t>(e[2]);
      if (power > proto::kMaxPower || (dir != 1 && dir != -1)) {
        faults_ |= proto::fault::kBadSet;
        ESP_LOGW(TAG, "SET rejected: ch%u power %u dir %d",
          static_cast<unsigned>(i), power, dir);
        return;
      }
      if (power > kCommandLimit) {
        power = kCommandLimit;
        faults_ |= proto::fault::kClamped;
      }
      cmd[i] = {power, static_cast<Direction>(dir)};
    }

    ack_seq_ = f.seq;
    last_valid_set_us_ = esp_timer_get_time();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      drive(i, cmd[i]);
    }
    applied_ = cmd;

    if (last_valid_set_us_ >= next_set_log_us_) {
      next_set_log_us_ = last_valid_set_us_ + kSetLogPeriodUs;
      ESP_LOGI(TAG, "SET seq=%3u  ch0 %5u %s | ch1 %5u %s", f.seq,
        applied_[0].power, proto::to_string(applied_[0].direction),
        applied_[1].power, proto::to_string(applied_[1].direction));
    }
  }

  // Boot lines are usually lost: USB re-enumerates after reset before the monitor reconnects.
  // A periodic line shows the firmware runs and whether the Pi is talking to it.
  void log_heartbeat(int64_t now) const
  {
    ESP_LOGI(TAG, "up %lld s, last reset: %s", static_cast<long long>(now / 1'000'000),
      reset_reason_name(esp_reset_reason()));
    if (last_valid_set_us_ == 0) {
      ESP_LOGI(TAG, "alive: %s, faults 0x%02X, rx frames %lu, crc errors %u, no SET yet",
        state_name(state_), faults_, static_cast<unsigned long>(rx_frames_), rx_crc_errors_);
    } else {
      ESP_LOGI(TAG, "alive: %s, faults 0x%02X, rx frames %lu, crc errors %u, last SET %lld ms ago",
        state_name(state_), faults_, static_cast<unsigned long>(rx_frames_), rx_crc_errors_,
        static_cast<long long>((now - last_valid_set_us_) / 1000));
    }
  }

  void change_state(DeviceState next, const char * reason)
  {
    if (next != state_) {
      ESP_LOGI(TAG, "%s -> %s (%s)", state_name(state_), state_name(next), reason);
      state_ = next;
    }
    if (next != DeviceState::Enabled) {
      stop_all();  // immediately, no ramp
    }
  }

  void stop_all()
  {
    applied_.fill({});
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      drive(i, applied_[i]);
    }
  }

  void send(MessageType type, std::span<const uint8_t> payload)
  {
    std::array<uint8_t, proto::kMaxFrameSize> out{};
    const std::size_t n = proto::encode_frame(type, tx_seq_++, payload, out);
    if (n > 0) {
      uart_write_bytes(kUart, out.data(), n);
    }
  }

  void send_info()
  {
    std::array<uint8_t, proto::kInfoSize> p{};
    p[0] = proto::kProtocolVersion;
    p[1] = static_cast<uint8_t>(kMotorCount);
    put_u16(&p[2], kCommandLimit);
    put_u16(&p[4], kWatchdogMs);
    p[6] = kFwVersion;
    send(MessageType::Info, p);
  }

  void send_status()
  {
    constexpr std::size_t kLen = proto::kStatusHeaderSize + kMotorCount * proto::kSetEntrySize;
    std::array<uint8_t, kLen> p{};
    p[0] = static_cast<uint8_t>(state_);
    p[1] = faults_;
    p[2] = ack_seq_;
    put_u16(&p[3], rx_crc_errors_);
    put_u16(&p[5], 0);  // battery_mv: not measured on the bench
    p[7] = static_cast<uint8_t>(kMotorCount);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      uint8_t * e = &p[proto::kStatusHeaderSize + i * proto::kSetEntrySize];
      put_u16(e, applied_[i].power);
      e[2] = static_cast<uint8_t>(applied_[i].direction);
    }
    send(MessageType::Status, p);
    next_status_us_ = esp_timer_get_time() + kStatusPeriodUs;
  }

  proto::FrameParser parser_;
  DeviceState state_{DeviceState::Disabled};
  uint8_t faults_{0};
  uint8_t ack_seq_{0};
  uint8_t tx_seq_{0};
  uint16_t rx_crc_errors_{0};
  std::array<MotorCommand, kMotorCount> applied_{};
  int64_t last_valid_set_us_{0};
  int64_t next_status_us_{0};
  int64_t next_set_log_us_{0};
  int64_t next_heartbeat_us_{0};
  uint32_t rx_frames_{0};
};
}  // namespace

extern "C" void app_main(void)
{
  static MotorController controller;  // static: keep it off the small app_main stack
  controller.init();                   // motors at 0 from here on

  if (!self_test()) {
    ESP_LOGE(TAG, "protocol self test FAILED: codec does not match the spec test vectors");
    return;  // never answer the Pi with a broken codec; motors stay at 0
  }
  ESP_LOGI(TAG, "protocol self test OK");

  controller.run();
}
