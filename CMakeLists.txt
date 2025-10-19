cmake_minimum_required(VERSION 3.16)
project(PiPlayground CXX)

# Abseil requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Download Abseil using FetchContent
include(FetchContent)
FetchContent_Declare(
  abseil-cpp
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        20250814.1 # You can update this tag to a newer stable release
)
FetchContent_MakeAvailable(abseil-cpp)

# 2. Define your executable
add_executable(main main.cc)

# 3. Link against the Abseil components you need
# You link against specific targets, e.g., absl::strings,
# absl::container, absl::time, etc.
target_link_libraries(main PRIVATE
  # absl::strings
  # absl::flat_hash_map
  absl::time
  pigpio
  pthread
  rt
)
