#ifndef SENSOR_MODULE_H_
#define SENSOR_MODULE_H_

#include <pigpio.h>

#include <expected>
#include <functional>
#include <utility>

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
  SensorModuleHandler(int scl_pin, int sda_pin, int int_pin, int drdy_pin)
      : scl_pin_(scl_pin),
        sda_pin_(sda_pin),
        int_pin_(int_pin),
        drdy_pin_(drdy_pin) {}

  ~SensorModuleHandler() {
    if (mpu6050_handle_ >= 0) {
      i2cClose(mpu6050_handle_);
    }
    if (ms5611_handle_ >= 0) {
      i2cClose(ms5611_handle_);
    }
  };

  int Init(bool calibrate_acc);

  std::expected<AccGyroData, int> ReadAccGyroData();

 private:
  bool LoadAccCalibration();
  std::pair<float, float> CalculateScaleAndOffset(
      std::function<float(const AccGyroData&)>);
  void CalibrateAcc();
  void CalibrateGyro();

  int scl_pin_ = -1;
  int sda_pin_ = -1;
  int int_pin_ = -1;
  int drdy_pin_ = -1;

  int mpu6050_handle_ = -1;
  int ms5611_handle_ = -1;

  float ax_scale_ = 1.0f;
  float ay_scale_ = 1.0f;
  float az_scale_ = 1.0f;
  float ax_offset_ = 0.0f;
  float ay_offset_ = 0.0f;
  float az_offset_ = 0.0f;

  float gx_offset_ = 0.0f;
  float gy_offset_ = 0.0f;
  float gz_offset_ = 0.0f;
};

}  // namespace rpi

#endif  // SENSOR_MODULE_H_
