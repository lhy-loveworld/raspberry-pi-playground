#include "remote_control.h"

#include <pigpio.h>

#include <algorithm>
#include <iostream>

namespace rpi {
int RemoteControlHandler::Init() {
  if (gpioInitialise() < 0) {
    std::cerr
        << "Failed to initialize pigpio. Is the pigpiod daemon running? Try "
           "running: sudo systemctl start pigpiod"
        << std::endl;
    return -1;
  }
  std::cout << "pigpio initialized successfully." << std::endl;
  for (const auto& [channel, pin] : channel_to_pin_) {
    gpioSetMode(pin, PI_INPUT);
    gpioSetAlertFuncEx(pin, WrapperOnEdge, this);
  }

  return 0;
}

float RemoteControlHandler::GetChannelPulseWidthPercentage(
    Channel channel) const {
  uint32_t pulse_width =
      last_pulse_widths_[static_cast<int>(channel) - 1].load();
  pulse_width = std::clamp(pulse_width, kMinPulseWidth, kMaxPulseWidth);
  return static_cast<float>(pulse_width - kMinPulseWidth) /
         (kMaxPulseWidth - kMinPulseWidth);
}

void RemoteControlHandler::WrapperOnEdge(int pin, int level, uint32_t tick,
                                         void* user_data) {
  static_cast<RemoteControlHandler*>(user_data)->OnEdge(pin, level, tick);
}

void RemoteControlHandler::OnEdge(const int pin, const int level,
                                  const uint32_t tick) {
  int channel_index = static_cast<int>(pin_to_channel_.at(pin)) - 1;
  if (level == 1) {
    last_ticks_[channel_index].store(tick);
  } else if (level == 0) {
    uint32_t last_tick = last_ticks_[channel_index].load();
    if (last_tick == 0) {
      return;
    }
    uint32_t pulse_width = tick - last_tick;
    last_pulse_widths_[channel_index].store(pulse_width);
  }
}

}  // namespace rpi
