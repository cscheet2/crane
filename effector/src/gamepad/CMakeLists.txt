cmake_minimum_required(VERSION 3.8)
project(gamepad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Build the hardware plugin
add_library(gamepad_hardware SHARED
  src/gamepad_hardware.cpp
)
target_include_directories(gamepad_hardware PUBLIC include)
ament_target_dependencies(gamepad_hardware
  hardware_interface
  rclcpp
  pluginlib
)

# Build the controller
add_library(gamepad_publisher SHARED
  src/gamepad_publisher.cpp
)
target_include_directories(gamepad_publisher PUBLIC include)
ament_target_dependencies(gamepad_publisher
  controller_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

# Export plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

# Install targets
install(
  TARGETS gamepad_hardware gamepad_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()