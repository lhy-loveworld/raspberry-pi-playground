#include "sensor_module.h"

#include <pigpio.h>

#include <chrono>
#include <expected>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

namespace rpi {
namespace {

constexpr char kAccCalibFilePath[] =
    "~/.config/flight_control/acc_calibration.txt";

}  // namespace

int SensorModuleHandler::Init(const bool calibrate_acc) {
  static constexpr int kI2cBusId = 1;
  static constexpr int kMpu6050Addr = 0x68;
  static constexpr int kMpu6050PwrMgmt1Reg = 0x6B;
  static constexpr int kMs5611Addr = 0x77;

  mpu6050_handle_ = i2cOpen(kI2cBusId, kMpu6050Addr, 0);
  if (mpu6050_handle_ < 0) {
    std::cerr << "Failed to open I2C for MPU6050." << std::endl;
    return mpu6050_handle_;
  }
  int ret = i2cWriteByteData(mpu6050_handle_, kMpu6050PwrMgmt1Reg, 0x00);
  if (ret < 0) {
    std::cerr << "Failed to wake up MPU6050." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 initialized successfully." << std::endl;

  if (!calibrate_acc) {
    if (!LoadAccCalibration()) {
      std::cerr << "Loading accelerometer calibration failed. Starting "
                   "re-calibration."
                << std::endl;
      CalibrateAcc();
    }
  } else {
    CalibrateAcc();
  }

  CalibrateGyro();

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
  if (mpu6050_handle_ < 0) {
    throw std::logic_error("MPU6050 not initialized");
  }
  char buf[kAccGyroDataLength] = {0};
  int ret = i2cReadI2CBlockData(mpu6050_handle_, kAccGyroStartReg, buf,
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
  auto combine_bytes = [](char high, char low) {
    return static_cast<int16_t>((static_cast<uint8_t>(high) << 8) |
                                static_cast<uint8_t>(low));
  };

  int16_t rawAx = combine_bytes(buf[0], buf[1]);
  int16_t rawAy = combine_bytes(buf[2], buf[3]);
  int16_t rawAz = combine_bytes(buf[4], buf[5]);
  // Skip temp: buf[6] and buf[7]
  int16_t rawGx = combine_bytes(buf[8], buf[9]);
  int16_t rawGy = combine_bytes(buf[10], buf[11]);
  int16_t rawGz = combine_bytes(buf[12], buf[13]);

  // Convert raw values to units
  // Default settings:
  // Accel sensitivity: +/- 2g -> 16384 LSB/g
  // Gyro sensitivity: +/- 250 dps -> 131 LSB/dps
  data.ax = (((float)rawAx / 16384.0f) - ax_offset_) * ax_scale_;
  data.ay = (((float)rawAy / 16384.0f) - ay_offset_) * ay_scale_;
  data.az = (((float)rawAz / 16384.0f) - az_offset_) * az_scale_;
  data.gx = (float)rawGx / 131.0f - gx_offset_;
  data.gy = (float)rawGy / 131.0f - gy_offset_;
  data.gz = (float)rawGz / 131.0f - gz_offset_;

  return data;
}

bool SensorModuleHandler::LoadAccCalibration() {
  std::ifstream file(kAccCalibFilePath);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << kAccCalibFilePath << "." << std::endl;
    return false;
  }
  if (file >> ax_scale_ >> ay_scale_ >> az_scale_ >> ax_offset_ >> ay_offset_ >>
      az_offset_) {
    std::cout << "Loaded accelerometer calibration." << std::endl;
    return true;
  }
  std::cerr << "Failed to read calibration data from acc_calibration.txt. File "
               "may be corrupted."
            << std::endl;
  return false;
}

std::pair<float, float> SensorModuleHandler::CalculateScaleAndOffset(
    std::function<float(const AccGyroData&)> sample_func) {
  constexpr int kSamples = 1000;
  double sum = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      sum += sample_func(data_result.value());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  double mean_pos = sum / kSamples;
  std::cout << "Mean positive reading: " << mean_pos << std::endl;
  std::cout << "Now flip the device vertically and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  sum = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      sum += sample_func(data_result.value());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  double mean_neg = sum / kSamples;
  std::cout << "Mean negative reading: " << mean_neg << std::endl;
  float scale = static_cast<float>((mean_pos - mean_neg) / 2.0);
  float offset = static_cast<float>((mean_pos + mean_neg) / 2.0);
  return {scale, offset};
}

void SensorModuleHandler::CalibrateAcc() {
  std::cout << "Calibrating accelerometer...Please follow the instructions."
            << std::endl;
  ax_scale_ = 1.0f;
  ay_scale_ = 1.0f;
  az_scale_ = 1.0f;
  ax_offset_ = 0.0f;
  ay_offset_ = 0.0f;
  az_offset_ = 0.0f;
  std::cout << "Place the device with Z axis pointing up and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  std::tie(az_scale_, az_offset_) =
      CalculateScaleAndOffset([](const AccGyroData& data) { return data.az; });
  std::cout << "Place the device with Y axis pointing up and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  std::tie(ay_scale_, ay_offset_) =
      CalculateScaleAndOffset([](const AccGyroData& data) { return data.ay; });
  std::cout << "Place the device with X axis pointing up and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  std::tie(ax_scale_, ax_offset_) =
      CalculateScaleAndOffset([](const AccGyroData& data) { return data.ax; });
  std::cout << "Accelerometer calibration complete: "
            << "ax_scale: " << ax_scale_ << ", ay_scale: " << ay_scale_
            << ", az_scale: " << az_scale_ << ", ax_offset: " << ax_offset_
            << ", ay_offset: " << ay_offset_ << ", az_offset: " << az_offset_
            << std::endl;
  std::cout << "Place the device back to normal orientation and press Enter.";
  std::cin.get();
  std::ofstream file(kAccCalibFilePath);
  if (file.is_open()) {
    file << ax_scale_ << " " << ay_scale_ << " " << az_scale_ << " "
         << ax_offset_ << " " << ay_offset_ << " " << az_offset_ << std::endl;
    std::cout << "Saved accelerometer calibration to " << kAccCalibFilePath
              << "." << std::endl;
  } else {
    std::cerr << "Failed to save accelerometer calibration to "
              << kAccCalibFilePath << "." << std::endl;
  }
}

void SensorModuleHandler::CalibrateGyro() {
  std::cout << "Calibrating gyroscope. Keep the device stationary for a few "
               "seconds..."
            << std::endl;

  constexpr int kSamples = 1000;
  double gx_sum = 0.0;
  double gy_sum = 0.0;
  double gz_sum = 0.0;

  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      gx_sum += data_result->gx;
      gy_sum += data_result->gy;
      gz_sum += data_result->gz;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  gx_offset_ = static_cast<float>(gx_sum / kSamples);
  gy_offset_ = static_cast<float>(gy_sum / kSamples);
  gz_offset_ = static_cast<float>(gz_sum / kSamples);

  std::cout << "Gyroscope calibration complete. Offsets - gx: " << gx_offset_
            << ", gy: " << gy_offset_ << ", gz: " << gz_offset_ << std::endl;
}

}  // namespace rpi
