#ifndef DATA_STRUCTS_H_
#define DATA_STRUCTS_H_

#include <cstdint>

namespace rpi {

// A struct to hold the remote control data for each channel.
// Roll, Pitch, Yaw: -1.0 to 1.0 (0.0 is center).
// Throttle: 0.0 to 1.0 (0.0 is min).
struct RcData {
  double roll = 0.0;
  double pitch = 0.0;
  double throttle = 0.0;
  double yaw = 0.0;
};

struct AccGyroData {
  int16_t raw_ax = 0;
  int16_t raw_ay = 0;
  int16_t raw_az = 0;
  int16_t raw_gx = 0;
  int16_t raw_gy = 0;
  int16_t raw_gz = 0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
};

}  // namespace rpi

#endif  // DATA_STRUCTS_H_
