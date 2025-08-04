cmake_minimum_required(VERSION 3.8)
project(cmeresearch_stepper_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(cmeresearch_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
        include
)

SET(TINKER_HEADER_FILES
        src/tinkerforge/ip_connection.h
        include/tinkerforge/bricklet_silent_stepper_v2.h
)

set(HEADER_FILES
        include/cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp
        include/cmeresearch_stepper_driver/stepper_driver_ros_wrapper.hpp)

add_library(tink_bindings src/tinkerforge/ip_connection.c src/tinkerforge/bricklet_silent_stepper_v2.c ${TINKER_HEADER_FILES})

add_library(stepper_driver src/tinkerforge_bricklet_stepper_driver.cpp ${HEADER_FILES} ${TINKER_HEADER_FILES})
ament_target_dependencies(stepper_driver rclcpp cmeresearch_msgs control_msgs)
target_link_libraries(stepper_driver tink_bindings)

add_library(stepper_driver_wrapper src/stepper_driver_ros_wrapper.cpp ${HEADER_FILES} ${TINKER_HEADER_FILES})
ament_target_dependencies(stepper_driver_wrapper rclcpp cmeresearch_msgs control_msgs)
target_link_libraries(stepper_driver_wrapper stepper_driver tink_bindings)

add_executable(stepper_driver_node src/stepper_driver_ros_wrapper_node.cpp ${HEADER_FILES} ${TINKER_HEADER_FILES})
ament_target_dependencies(stepper_driver_node rclcpp cmeresearch_msgs control_msgs)
target_link_libraries(stepper_driver_node stepper_driver_wrapper stepper_driver tink_bindings)

#ament_export_targets(stepper_driver HAS_LIBRARY_TARGET)
#ament_export_targets(stepper_driver_node HAS_LIBRARY_TARGET)

install(
        TARGETS stepper_driver_node
        EXPORT stepper_driver_node
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include stepper_driver_node
)

install(
        TARGETS stepper_driver
        EXPORT stepper_driver
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include stepper_driver
)

install(
        TARGETS stepper_driver_wrapper
        EXPORT stepper_driver_wrapper
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include stepper_driver_wrapper
)

install(
        TARGETS tink_bindings
        EXPORT tink_bindings
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include tinkerforge
)

install(TARGETS
    stepper_driver
    stepper_driver_node
    stepper_driver_wrapper
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

