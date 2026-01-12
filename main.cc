#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <format>
#include <iostream>
#include <string>

#include "data_structs.h"
#include "remote_control.h"
#include "sensor_module.h"

namespace {

std::atomic<bool> g_running = true;

constexpr char kDebugFlag[] = "--debug";
constexpr char kCalibrateAccFlag[] = "--calibrate-acc";

// GPIO pin definitions for remote control channels.
constexpr int kCh1Pin = 26;
constexpr int kCh2Pin = 22;
constexpr int kCh3Pin = 27;
constexpr int kCh4Pin = 17;

// GPIO pin definitions for sensors.
constexpr int kSensorSclPin = 3;
constexpr int kSensorSdaPin = 2;
constexpr int kSensorIntPin = 6;
constexpr int kSensorDrdyPin = 5;

constexpr int kLoopDelayUs = 2000;
constexpr int kPrintDelayMs = 100;

void SignalHandler(int sig_num) { g_running = false; }

void PrintRcData(const rpi::RcData& data) {
  std::cout << std::format("\r\033[2KRoll: {:.2f}\n", data.roll);
  std::cout << std::format("\r\033[2KPitch: {:.2f}\n", data.pitch);
  std::cout << std::format("\r\033[2KThrottle: {:.2f}\n", data.throttle);
  std::cout << std::format("\r\033[2KYaw: {:.2f}\n", data.yaw);
}

void PrintSensorData(const rpi::AccGyroData& data) {
  std::cout << std::format(
      "\r\033[2KAcc (g): ax={:.2f}, ay={:.2f}, az={:.2f}\n", data.ax, data.ay,
      data.az);
  std::cout << std::format(
      "\r\033[2KGyro (dps): gx={:.2f}, gy={:.2f}, gz={:.2f}\n", data.gx,
      data.gy, data.gz);
}

}  // namespace

int main(int argc, char* argv[]) {
  std::cout << "Program started. Use Ctrl+C to exit..." << std::endl;

  bool debug = false;
  bool calibrate_acc = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == kDebugFlag) {
      debug = true;
    }
    if (std::string(argv[i]) == kCalibrateAccFlag) {
      calibrate_acc = true;
    }
  }

  if (int ret = gpioInitialise(); ret < 0) {
    std::cerr << "Failed to initialize pigpio." << std::endl;
    return ret;
  }
  std::cout << "pigpio initialized successfully." << std::endl;
  atexit(gpioTerminate);

  rpi::RemoteControlHandler rc_handler(kCh1Pin, kCh2Pin, kCh3Pin, kCh4Pin);
  if (int ret = rc_handler.Init(); ret != 0) {
    std::cerr << "Failed to initialize Remote Control Handler." << std::endl;
    return ret;
  };
  std::cout << "Remote Control Handler initialized successfully." << std::endl;

  rpi::SensorModuleHandler sensor_module_handler(kSensorSclPin, kSensorSdaPin,
                                                 kSensorIntPin, kSensorDrdyPin);
  if (int ret = sensor_module_handler.Init(calibrate_acc); ret != 0) {
    std::cerr << "Failed to initialize Sensor Module Handler." << std::endl;
    return ret;
  };
  std::cout << "Sensor Module initialized successfully." << std::endl;

  gpioSetSignalFunc(SIGINT, SignalHandler);

  std::chrono::steady_clock::time_point next_loop_time =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point next_print_time =
      std::chrono::steady_clock::now();

  do {
    next_loop_time += std::chrono::microseconds(kLoopDelayUs);
    std::chrono::steady_clock::time_point current_time =
        std::chrono::steady_clock::now();

    // Busy wait to maintain precise loop timing.
    do {
      current_time = std::chrono::steady_clock::now();
    } while (current_time < next_loop_time);

    std::expected<rpi::AccGyroData, int> acc_gyro_data_result =
        sensor_module_handler.ReadAccGyroData();
    if (!acc_gyro_data_result.has_value()) {
      return acc_gyro_data_result.error();
    }
    const rpi::AccGyroData& data = *acc_gyro_data_result;
    rpi::RcData rc_data = rc_handler.GetRcData();

    if (debug && current_time >= next_print_time) {
      PrintRcData(rc_data);
      PrintSensorData(data);
      std::cout.flush();
      std::cout << "\033[6A";
      next_print_time += std::chrono::milliseconds(kPrintDelayMs);
    }
  } while (g_running.load());

  std::cout << "\n\n\n\n\n\n" << std::endl;
  std::cout << "Ctrl+C received. Shutting down gracefully..." << std::endl;
  return 0;
}
