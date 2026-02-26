cmake_minimum_required(VERSION 3.16)
project(simulink_to_cpp)

set(CMAKE_CXX_STANDARD 17)

# Legacy C code
add_library(legacy_pid    legacy/pid_controller.c)
add_library(legacy_kalman legacy/kalman_filter.c)

# S-Function bridge (C++ that wraps the new C++ class)
add_library(pid_bridge bridge/pid_sfunc_bridge.cpp)
target_include_directories(pid_bridge PUBLIC include)

# Test: PID equivalence
add_executable(test_pid tests/test_pid_equivalence.cpp)
target_include_directories(test_pid PRIVATE include legacy)
target_link_libraries(test_pid legacy_pid)

# Test: Kalman equivalence
add_executable(test_kalman tests/test_kalman_equivalence.cpp)
target_include_directories(test_kalman PRIVATE include legacy)
target_link_libraries(test_kalman legacy_kalman)

# Run all tests with: cmake --build build && ctest --test-dir build
enable_testing()
add_test(NAME PID_Equivalence    COMMAND test_pid)
add_test(NAME Kalman_Equivalence COMMAND test_kalman)
