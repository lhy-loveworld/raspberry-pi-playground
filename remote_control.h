#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

#include <pigpio.h>

#include <array>
#include <atomic>
#include <unordered_map>

namespace rpi {

enum class Channel { ROLL = 1, PITCH = 2, THROTTLE = 3, YAW = 4 };

// A struct to hold the remote control data for each channel.
// Roll, Pitch, Yaw: -1.0 to 1.0 (0.0 is center).
// Throttle: 0.0 to 1.0 (0.0 is min).
struct RcData {
  float roll;
  float pitch;
  float throttle;
  float yaw;
};

// A handler to process the channels from FS-T6 remote controller.
class RemoteControlHandler {
 public:
  int Init();

  RcData GetRcData() const {
    return RcData{
        .roll = GetChannelNormalized(Channel::ROLL),
        .pitch = GetChannelNormalized(Channel::PITCH),
        .throttle = GetChannelNormalized(Channel::THROTTLE),
        .yaw = GetChannelNormalized(Channel::YAW),
    };
  }

  float GetChannelNormalized(Channel channel) const;

  // Roll, Pitch, Throttle, Yaw -> Channel 1, 2, 3, 4
  RemoteControlHandler(const int roll_pin, const int pitch_pin,
                       const int throttle_pin, const int yaw_pin)
      : channel_to_pin_{{Channel::ROLL, roll_pin},
                        {Channel::PITCH, pitch_pin},
                        {Channel::THROTTLE, throttle_pin},
                        {Channel::YAW, yaw_pin}},
        pin_to_channel_{{roll_pin, Channel::ROLL},
                        {pitch_pin, Channel::PITCH},
                        {throttle_pin, Channel::THROTTLE},
                        {yaw_pin, Channel::YAW}} {}

  ~RemoteControlHandler() = default;

 private:
  // Static wrapper for C style callback.
  static void WrapperOnEdge(int pin, int level, uint32_t tick, void* user_data);
  // Callback function for GPIO edge detection.
  void OnEdge(int pin, int level, uint32_t tick);

  static constexpr uint32_t kMinPulseWidth = 1000;
  static constexpr uint32_t kMaxPulseWidth = 2000;

  // GPIO pin ID for 4 channels.
  const std::unordered_map<Channel, int> channel_to_pin_;
  const std::unordered_map<int, Channel> pin_to_channel_;

  std::array<std::atomic<uint32_t>, 4> last_ticks_{0, 0, 0, 0};

  std::array<std::atomic<uint32_t>, 4> last_pulse_widths_{0, 0, 0, 0};
};
}  // namespace rpi

#endif  // REMOTE_CONTROL_H_
