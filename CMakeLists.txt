cmake_minimum_required(VERSION 3.16)
project(PiPlayground CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(main main.cc remote_control.cc sensor_module.cc)

target_link_libraries(main PRIVATE
  pigpio
  pthread
  rt
)
