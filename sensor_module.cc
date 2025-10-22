#include "sensor_module.h"

#include <expected>
#include <iostream>

#include <pigpio.h>

namespace rpi {

int SensorModuleHandler::Init() {
  static constexpr int kI2cBusId = 1;
  static constexpr int kMpu6050Addr = 0x68;
  static constexpr int kMpu6050PwrMgmt1Reg = 0x6B;
  static constexpr int kMs5611Addr = 0x77;

  mpu6060_handle_ = i2cOpen(kI2cBusId, kMpu6050Addr, 0);
  if (mpu6060_handle_ < 0) {
    std::cerr << "Failed to open I2C for MPU6050." << std::endl;
    return mpu6060_handle_;
  }
  int ret = i2cWriteByteData(mpu6060_handle_, kMpu6050PwrMgmt1Reg, 0x00);
  if (ret < 0) {
    std::cerr << "Failed to wake up MPU6050." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 initialized successfully." << std::endl;

  // ms5611_handle_ = i2cOpen(kI2cBusId, kMs5611Addr, 0);
  // if (ms5611_handle_ < 0) {
  //   std::cerr << "Failed to open I2C for MS5611." << std::endl;
  //   return ms5611_handle_;
  // }
  return 0;
}

std::expected<AccGyroData, int> SensorModuleHandler::ReadAccGyroData() {
  static constexpr int kAccGyroStartReg = 0x3B;
  static constexpr int kAccGyroDataLength = 14;
  AccGyroData data{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  if (mpu6060_handle_ < 0) {
    throw std::logic_error("MPU6050 not initialized");
  }
  char buf[kAccGyroDataLength] = {0};
  int ret = i2cReadI2CBlockData(mpu6060_handle_, kAccGyroStartReg, buf,
                                kAccGyroDataLength);
  if (ret < 0) {
    std::cerr << "Failed to read Acc/Gyro data from MPU6050." << std::endl;
    return std::unexpected(ret);
  }
  if (ret != kAccGyroDataLength) {
    throw std::runtime_error("Incomplete read from MPU6050");
  }
  // Combine high and low bytes for each value
  // These are 16-bit two's complement values
  int16_t rawAx = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawAy = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawAz = (int16_t)((buf[4] << 8) | buf[5]);
  // Skip temp: buf[6] and buf[7]
  int16_t rawGx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]);

  // Convert raw values to units
  // Default settings:
  // Accel sensitivity: +/- 2g -> 16384 LSB/g
  // Gyro sensitivity: +/- 250 dps -> 131 LSB/dps
  data.ax = (float)rawAx / 16384.0f;
  data.ay = (float)rawAy / 16384.0f;
  data.az = (float)rawAz / 16384.0f;
  data.gx = (float)rawGx / 131.0f;
  data.gy = (float)rawGy / 131.0f;
  data.gz = (float)rawGz / 131.0f;

  return data;
}

}  // namespace rpi
