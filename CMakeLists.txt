cmake_minimum_required(VERSION 3.8)
project(cmeresearch_stepper_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmeresearch_msgs REQUIRED)
find_package(Threads REQUIRED)

set(WARNINGS -Wall -Wextra -Wpedantic)

# --- Vendored TinkerForge C bindings (CC0, see tfextern/license.txt) ---------
# Only the two units needed for the SilentStepper Bricklet 2.0 are kept.
# Built without our warning flags: this is third-party code.
add_library(tink_bindings STATIC
  src/tinkerforge/ip_connection.c
  src/tinkerforge/bricklet_silent_stepper_v2.c)
target_include_directories(tink_bindings PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tinkerforge)
set_target_properties(tink_bindings PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(tink_bindings PUBLIC Threads::Threads)

# --- Driver logic (no ROS, no TinkerForge: unit-testable with a fake BrickletIo)
add_library(stepper_driver_core STATIC
  src/tinkerforge_bricklet_stepper_driver.cpp)
target_include_directories(stepper_driver_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(stepper_driver_core PRIVATE ${WARNINGS})
target_link_libraries(stepper_driver_core PUBLIC Threads::Threads)

# --- TinkerForge BrickletIo implementation -----------------------------------
add_library(tinkerforge_io STATIC src/tinkerforge_io.cpp)
target_compile_options(tinkerforge_io PRIVATE ${WARNINGS})
target_link_libraries(tinkerforge_io PUBLIC stepper_driver_core tink_bindings)

# --- ROS node ----------------------------------------------------------------
add_library(stepper_driver_ros STATIC src/stepper_driver_ros_wrapper.cpp)
target_compile_options(stepper_driver_ros PRIVATE ${WARNINGS})
target_link_libraries(stepper_driver_ros PUBLIC
  tinkerforge_io
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${cmeresearch_msgs_TARGETS})

add_executable(stepper_driver_node src/stepper_driver_ros_wrapper_node.cpp)
target_compile_options(stepper_driver_node PRIVATE ${WARNINGS})
target_link_libraries(stepper_driver_node stepper_driver_ros)

install(TARGETS stepper_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stepper_math test/test_stepper_math.cpp)
  target_include_directories(test_stepper_math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Driver logic against a fake BrickletIo: reconnects, stale data, config errors.
  ament_add_gtest(test_stepper_driver test/test_stepper_driver.cpp TIMEOUT 60)
  target_link_libraries(test_stepper_driver stepper_driver_core)

  # Node-level test: runs stepper_driver_node in hw_simulation mode.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_node_simulation.launch.py TIMEOUT 120)
endif()

ament_package()
