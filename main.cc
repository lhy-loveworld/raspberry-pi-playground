#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <format>
#include <iostream>
#include <thread>

#include "remote_control.h"
#include "sensor_module.h"

std::atomic<bool> g_running = true;

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

namespace {
void SignalHandler(int sig_num) { g_running = false; }

void PrintChannelOutput(const rpi::RemoteControlHandler& rc_handler) {
  std::cout << std::format(
      "\r\033[2KRoll: {:.1f}%\n",
      rc_handler.GetChannelPulseWidthPercentage(rpi::Channel::ROLL) * 100);
  std::cout << std::format(
      "\r\033[2KPitch: {:.1f}%\n",
      rc_handler.GetChannelPulseWidthPercentage(rpi::Channel::PITCH) * 100);
  std::cout << std::format(
      "\r\033[2KThrottle: {:.1f}%\n",
      rc_handler.GetChannelPulseWidthPercentage(rpi::Channel::THROTTLE) * 100);
  std::cout << std::format(
      "\r\033[2KYaw: {:.1f}%\n",
      rc_handler.GetChannelPulseWidthPercentage(rpi::Channel::YAW) * 100);
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

  if (int ret = gpioInitialise(); ret < 0) {
    std::cerr
        << "Failed to initialize pigpio. Is the pigpiod daemon running? Try "
           "running: sudo systemctl start pigpiod"
        << std::endl;
    return ret;
  }
  std::cout << "pigpio initialized successfully." << std::endl;
  atexit(gpioTerminate);

  gpioSetSignalFunc(SIGINT, SignalHandler);

  rpi::RemoteControlHandler rc_handler(kCh1Pin, kCh2Pin, kCh3Pin, kCh4Pin);
  if (int ret = rc_handler.Init(); ret != 0) {
    std::cerr << "Failed to initialize Remote Control Handler." << std::endl;
    return ret;
  };
  std::cout << "Remote Control Handler initialized successfully." << std::endl;

  rpi::SensorModuleHandler sensor_module_handler(kSensorSclPin, kSensorSdaPin,
                                                 kSensorIntPin, kSensorDrdyPin);
  if (int ret = sensor_module_handler.Init(); ret != 0) {
    std::cerr << "Failed to initialize Sensor Module Handler." << std::endl;
    return ret;
  };
  std::cout << "Sensor Module initialized successfully." << std::endl;

  do {
    PrintChannelOutput(rc_handler);
  
    auto acc_gyro_data_result = sensor_module_handler.ReadAccGyroData();
    if (!acc_gyro_data_result.has_value()) {
      return acc_gyro_data_result.error();
    }
    const rpi::AccGyroData& data = *acc_gyro_data_result;
    PrintSensorData(data);
    std::cout.flush();
    std::cout << "\033[6A";
  
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (g_running.load());

  std::cout << "\n\n\n\n\n\n" << std::endl;
  std::cout << "Ctrl+C received. Shutting down gracefully..." << std::endl;
  return 0;
}
