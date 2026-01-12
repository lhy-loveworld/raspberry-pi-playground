#include "sensor_module.h"

#include <pigpio.h>

#include <chrono>
#include <cstdlib>
#include <expected>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include "data_structs.h"

namespace rpi {
namespace {

std::string GetAccCalibFilePath() {
  const char* home = std::getenv("HOME");
  if (home == nullptr) {
    return "acc_calibration.txt";
  }
  return std::string(home) + "/.config/flight_control/acc_calibration.txt";
}

}  // namespace

int SensorModuleHandler::Init(const bool calibrate_acc) {
  static constexpr int kI2cBusId = 1;
  static constexpr int kMpu6050Addr = 0x68;
  static constexpr int kMpu6050SmplrtDivReg = 0x19;
  static constexpr int kMpu6050CfgReg = 0x1A;
  static constexpr int kMpu6050AccConfigReg = 0x1C;
  static constexpr int kMpu6050GyroConfigReg = 0x1B;
  static constexpr int kMpu6050PwrMgmt1Reg = 0x6B;
  static constexpr int kMs5611Addr = 0x77;
  // Need to enable MPU6050 I2C bypass to access HMC5883L directly.
  static constexpr int kHmc5883lAddr = 0x1E;

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

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Set clock source to PLL with X axis gyroscope reference
  ret = i2cWriteByteData(mpu6050_handle_, kMpu6050PwrMgmt1Reg, 0x01);
  if (ret < 0) {
    std::cerr << "Failed to set MPU6050 clock source." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 clock source set successfully to PLL with X axis "
               "gyroscope reference."
            << std::endl;

  // Accel config set to +/- 8g
  ret = i2cWriteByteData(mpu6050_handle_, kMpu6050AccConfigReg, 0x10);
  if (ret < 0) {
    std::cerr << "Failed to configure MPU6050 accelerometer." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 accelerometer configured successfully to +/- 8g."
            << std::endl;

  // Gyro config set to +/- 2000 dps
  ret = i2cWriteByteData(mpu6050_handle_, kMpu6050GyroConfigReg, 0x18);
  if (ret < 0) {
    std::cerr << "Failed to configure MPU6050 gyroscope." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 configured successfully to +/- 2000 dps." << std::endl;

  // Setup DLPF to 44Hz for Acc and 42Hz for Gyro
  ret = i2cWriteByteData(mpu6050_handle_, kMpu6050CfgReg, 0x03);
  if (ret < 0) {
    std::cerr << "Failed to set MPU6050 DLPF." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 DLPF set successfully to 44Hz for Acc and 42Hz for "
               "Gyro."
            << std::endl;

  // Set sample rate to 1kHz.
  ret = i2cWriteByteData(mpu6050_handle_, kMpu6050SmplrtDivReg, 0x00);
  if (ret < 0) {
    std::cerr << "Failed to set MPU6050 sample rate." << std::endl;
    return ret;
  }
  std::cout << "MPU6050 sample rate set successfully to 1kHz." << std::endl;

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
  AccGyroData data{};
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

  data.raw_ax = combine_bytes(buf[0], buf[1]);
  data.raw_ay = combine_bytes(buf[2], buf[3]);
  data.raw_az = combine_bytes(buf[4], buf[5]);
  // Skip temp: buf[6] and buf[7]
  data.raw_gx = combine_bytes(buf[8], buf[9]);
  data.raw_gy = combine_bytes(buf[10], buf[11]);
  data.raw_gz = combine_bytes(buf[12], buf[13]);

  // Convert raw values to units
  // Default settings:
  // Accel sensitivity: +/- 8g -> 4096 LSB/g
  // Gyro sensitivity: +/- 2000 dps -> 16.4 LSB/dps
  data.ax = (data.raw_ax - ax_offset_) / ax_scale_;
  data.ay = (data.raw_ay - ay_offset_) / ay_scale_;
  data.az = (data.raw_az - az_offset_) / az_scale_;
  data.gx = (data.raw_gx - gx_offset_) / 16.4;
  data.gy = (data.raw_gy - gy_offset_) / 16.4;
  data.gz = (data.raw_gz - gz_offset_) / 16.4;

  return data;
}

bool SensorModuleHandler::LoadAccCalibration() {
  const std::string path = GetAccCalibFilePath();
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << path << "." << std::endl;
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

std::pair<double, double> SensorModuleHandler::CalculateScaleAndOffset(
    std::function<int16_t(const AccGyroData&)> sample_func) {
  constexpr int kSamples = 1000;
  int32_t sum = 0;
  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      sum += sample_func(data_result.value());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  double mean_pos = static_cast<double>(sum) / kSamples;
  std::cout << "Mean positive reading: " << mean_pos << std::endl;
  std::cout << "Now flip the device vertically and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  sum = 0;
  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      sum += sample_func(data_result.value());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  double mean_neg = static_cast<double>(sum) / kSamples;
  std::cout << "Mean negative reading: " << mean_neg << std::endl;
  double scale = (mean_pos - mean_neg) / 2.0;
  double offset = (mean_pos + mean_neg) / 2.0;
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
  std::tie(az_scale_, az_offset_) = CalculateScaleAndOffset(
      [](const AccGyroData& data) { return data.raw_az; });
  std::cout << "Place the device with Y axis pointing up and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  std::tie(ay_scale_, ay_offset_) = CalculateScaleAndOffset(
      [](const AccGyroData& data) { return data.raw_ay; });
  std::cout << "Place the device with X axis pointing up and press Enter.";
  std::cin.get();
  std::cout << "Collecting data. Please wait..." << std::endl;
  std::tie(ax_scale_, ax_offset_) = CalculateScaleAndOffset(
      [](const AccGyroData& data) { return data.raw_ax; });
  std::cout << "Accelerometer calibration complete: "
            << "ax_scale: " << ax_scale_ << ", ay_scale: " << ay_scale_
            << ", az_scale: " << az_scale_ << ", ax_offset: " << ax_offset_
            << ", ay_offset: " << ay_offset_ << ", az_offset: " << az_offset_
            << std::endl;
  std::cout << "Place the device back to normal orientation and press Enter.";
  std::cin.get();
  const std::string path = GetAccCalibFilePath();
  std::filesystem::path fs_path(path);
  if (fs_path.has_parent_path()) {
    std::filesystem::create_directories(fs_path.parent_path());
  }
  std::ofstream file(path);
  if (file.is_open()) {
    file << ax_scale_ << " " << ay_scale_ << " " << az_scale_ << " "
         << ax_offset_ << " " << ay_offset_ << " " << az_offset_ << std::endl;
    std::cout << "Saved accelerometer calibration to " << path << "."
              << std::endl;
  } else {
    std::cerr << "Failed to save accelerometer calibration to " << path << "."
              << std::endl;
  }
}

void SensorModuleHandler::CalibrateGyro() {
  std::cout << "Calibrating gyroscope. Keep the device stationary for a few "
               "seconds..."
            << std::endl;

  constexpr int kSamples = 1000;
  int32_t gx_sum = 0;
  int32_t gy_sum = 0;
  int32_t gz_sum = 0;

  for (int i = 0; i < kSamples; ++i) {
    auto data_result = ReadAccGyroData();
    if (data_result.has_value()) {
      gx_sum += data_result->raw_gx;
      gy_sum += data_result->raw_gy;
      gz_sum += data_result->raw_gz;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  gx_offset_ = static_cast<double>(gx_sum) / kSamples;
  gy_offset_ = static_cast<double>(gy_sum) / kSamples;
  gz_offset_ = static_cast<double>(gz_sum) / kSamples;

  std::cout << "Gyroscope calibration complete. Offsets - gx: " << gx_offset_
            << ", gy: " << gy_offset_ << ", gz: " << gz_offset_ << std::endl;
}

}  // namespace rpi
