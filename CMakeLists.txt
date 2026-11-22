cmake_minimum_required(VERSION 3.8)
project(cmeresearch_stepper_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmeresearch_msgs REQUIRED)
find_package(Threads REQUIRED)

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

# --- Driver ------------------------------------------------------------------
add_library(stepper_driver STATIC
  src/tinkerforge_bricklet_stepper_driver.cpp
  src/stepper_driver_ros_wrapper.cpp)
target_include_directories(stepper_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(stepper_driver PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(stepper_driver PUBLIC tink_bindings)
ament_target_dependencies(stepper_driver PUBLIC rclcpp std_msgs cmeresearch_msgs)

add_executable(stepper_driver_node src/stepper_driver_ros_wrapper_node.cpp)
target_compile_options(stepper_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(stepper_driver_node stepper_driver)

install(TARGETS stepper_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stepper_math test/test_stepper_math.cpp)
  target_include_directories(test_stepper_math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
