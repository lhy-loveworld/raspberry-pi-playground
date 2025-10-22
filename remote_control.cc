#include "remote_control.h"

#include <pigpio.h>

#include <algorithm>
#include <iostream>

namespace rpi {
int RemoteControlHandler::Init() {
  for (const auto& [channel, pin] : channel_to_pin_) {
    int ret = gpioSetMode(pin, PI_INPUT);
    if (ret != 0) {
      std::cerr << "gpioSetMode failed for pin " << pin << " and channel "
                << static_cast<int>(channel) << std::endl;
      return ret;
    }
    ret = gpioSetAlertFuncEx(pin, WrapperOnEdge, this);
    if (ret != 0) {
      std::cerr << "gpioSetAlertFuncEx failed for pin " << pin << " and channel "
                << static_cast<int>(channel) << std::endl;
      return ret;
    }
  }
  std::cout << "Remote Control Handler initialized successfully." << std::endl;
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
