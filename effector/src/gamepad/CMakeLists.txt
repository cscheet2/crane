cmake_minimum_required(VERSION 3.8)
project(gamepad)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_plugin_parser REQUIRED)
find_package(controller_interface REQUIRED) # if you add a controller plugin

# Include headers
include_directories(
  include
)

# Hardware plugin
add_library(gamepad_hardware SHARED
  src/gamepad.cpp
)

ament_target_dependencies(gamepad_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# Pluginlib export
pluginlib_export_plugin_description_file(hardware_interface gamepad_hardware_plugin.xml)

# Install targets and headers
install(
  TARGETS gamepad_hardware
  EXPORT export_gamepad
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES
    config/gamepad_system.yaml
    launch/gamepad_launch.py
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES gamepad_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
