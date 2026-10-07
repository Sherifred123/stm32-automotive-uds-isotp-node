cmake_minimum_required(VERSION 3.16)
project(stm32_automotive_uds_isotp_node C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

# Host Simulation Executable
add_executable(uds_demo
    src/can_driver.c
    src/isotp.c
    src/uds_server.c
    src/diagnostic_app.c
    src/main.c
)

# Automated Unit Tests
enable_testing()
add_executable(uds_tests
    src/can_driver.c
    src/isotp.c
    src/uds_server.c
    tests/unity/unity.c
    tests/test_uds_isotp_suite.c
)
target_include_directories(uds_tests PRIVATE tests/unity)
add_test(NAME UDS_Protocol_Tests COMMAND uds_tests)
