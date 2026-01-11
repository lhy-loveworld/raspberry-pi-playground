cmake_minimum_required(VERSION 3.20)
project(PiPlayground CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(main main.cc remote_control.cc remote_control.h sensor_module.cc sensor_module.h)

target_link_libraries(main PRIVATE
  pigpio
  rt
  Threads::Threads
)
