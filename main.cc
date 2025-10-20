#include <atomic>
#include <csignal>
#include <iostream>

#include "absl/log/check.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "remote_control.h"

std::atomic<bool> g_running = true;

constexpr int kCh1Pin = 26;
constexpr int kCh2Pin = 22;
constexpr int kCh3Pin = 27;
constexpr int kCh4Pin = 17;

namespace {
void SignalHandler(int sig_num) {
  g_running = false;
}
}  // namespace

int main() {
  signal(SIGINT, SignalHandler);
  std::cout << "Program started. Use Ctrl+C to exit..." << std::endl;
  rpi::RemoteControlHandler rc_handler(kCh1Pin, kCh2Pin, kCh3Pin, kCh4Pin);
  QCHECK_OK(rc_handler.Init());
  while (g_running.load()) {
    absl::SleepFor(absl::Milliseconds(100));
  }
  std::cout << "\nCtrl+C received. Shutting down gracefully..." << std::endl;
  return 0;
}
