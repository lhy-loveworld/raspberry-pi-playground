#include <atomic>
#include <chrono>
#include <csignal>
#include <format>
#include <iostream>
// #include <string>
#include <thread>

#include "remote_control.h"

std::atomic<bool> g_running = true;

constexpr int kCh1Pin = 26;
constexpr int kCh2Pin = 22;
constexpr int kCh3Pin = 27;
constexpr int kCh4Pin = 17;

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
  std::cout.flush();
  std::cout << "\033[4A";
}
}  // namespace

int main(int argc, char* argv[]) {
  std::cout << "Program started. Use Ctrl+C to exit..." << std::endl;
  rpi::RemoteControlHandler rc_handler(kCh1Pin, kCh2Pin, kCh3Pin, kCh4Pin);
  if (rc_handler.Init() != 0) {
    std::cerr << "Failed to initialize Remote Control Handler." << std::endl;
    return -1;
  };
  signal(SIGINT, SignalHandler);
  do {
    PrintChannelOutput(rc_handler);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (g_running.load());
  std::cout << "\n\n\n\n" << std::endl;
  std::cout << "Ctrl+C received. Shutting down gracefully..." << std::endl;
  return 0;
}
