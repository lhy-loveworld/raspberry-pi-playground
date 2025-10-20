#include "remote_control.h"

#include <pigpio.h>

#include "absl/status/status.h"

namespace rpi {
absl::Status RemoteControlHandler::Init() {
  if (gpioInitialise() < 0) {
    return absl::InternalError(
        "Failed to initialize pigpio. Is the pigpiod daemon running? Try "
        "running: sudo systemctl start pigpiod");
  }
  // gpioSetMode(, PI_INPUT);
  return absl::OkStatus();
}
}  // namespace rpi
