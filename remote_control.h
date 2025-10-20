#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

#include <array>
#include <atomic>

#include <pigpio.h>

#include "absl/status/status.h"

namespace rpi {
// A handler to process the channels from FS-T6 remote controller.
class RemoteControlHandler {
 public:
  absl::Status Init();

  RemoteControlHandler(int ch1_pin, int ch2_pin, int ch3_pin, int ch4_pin)
      : ch_1_pin_(ch1_pin),
        ch_2_pin_(ch2_pin),
        ch_3_pin_(ch3_pin),
        ch_4_pin_(ch4_pin) {}

  ~RemoteControlHandler() { gpioTerminate(); }

 private:
  // GPIO pin ID for 4 channels.
  const int ch_1_pin_ = 1;
  const int ch_2_pin_ = 1;
  const int ch_3_pin_ = 1;
  const int ch_4_pin_ = 1;

  std::array<std::atomic<uint32_t>, 4> last_ticks_; 

  std::array<std::atomic<bool>, 4> valid_;
};
}  // namespace rpi

#endif // REMOTE_CONTROL_H_
