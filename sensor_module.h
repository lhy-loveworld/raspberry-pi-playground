#ifndef SENSOR_MODULE_H_
#define SENSOR_MODULE_H_

#include <expected>

#include <pigpio.h>

namespace rpi {

struct AccGyroData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

class SensorModuleHandler {
 public:
  int Init();

  std::expected<AccGyroData, int> ReadAccGyroData();
  
  SensorModuleHandler(int scl_pin, int sda_pin, int int_pin, int drdy_pin)
      : scl_pin_(scl_pin),
        sda_pin_(sda_pin),
        int_pin_(int_pin),
        drdy_pin_(drdy_pin) {}

  ~SensorModuleHandler() {
    if (mpu6060_handle_ >= 0) {
      i2cClose(mpu6060_handle_);
    }
    if (ms5611_handle_ >= 0) {
      i2cClose(ms5611_handle_);
    }
  };

 private:
  int scl_pin_;
  int sda_pin_;
  int int_pin_;
  int drdy_pin_;

  int mpu6060_handle_ = -1;
  int ms5611_handle_ = -1;
};

}  // namespace rpi

#endif  // SENSOR_MODULE_H_
