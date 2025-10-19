#include <atomic>
#include <csignal>
#include <iostream>
#include <pigpio.h>

#include "absl/time/clock.h"
#include "absl/time/time.h"

std::atomic<bool> g_running = true;

void SignalHandler(int sig_num) {
  g_running = false;
}

int main() {
  signal(SIGINT, SignalHandler);
  std::cout << "Program started. Use Ctrl+C to exit..." << std::endl;
  while (g_running.load()) {
    absl::SleepFor(absl::Milliseconds(100));
  }
  std::cout << "\nCtrl+C received. Shutting down gracefully..." << std::endl;
  return 0;
}
