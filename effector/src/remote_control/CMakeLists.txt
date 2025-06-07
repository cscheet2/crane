cmake_minimum_required(VERSION 3.8)
project(remote_control)

# Compiler options for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBEVDEV REQUIRED IMPORTED_TARGET libevdev)

# Add library for hardware interface plugin
add_library(remote_control SHARED
  src/remote_control.cpp
)

# Include directories for both builds and installs
target_include_directories(remote_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBEVDEV_INCLUDE_DIRS}
)

# Link libraries for hardware interface
target_link_libraries(remote_control
  PkgConfig::LIBEVDEV
)

# Declare dependencies for plugin
ament_target_dependencies(remote_control
  rclcpp
  sensor_msgs
  std_msgs
  hardware_interface
  pluginlib
)

# Export pluginlib XML
pluginlib_export_plugin_description_file(hardware_interface
  remote_control.xml
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install hardware interface plugin
install(
  TARGETS remote_control
  EXPORT export_remote_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install extra files like launch or description
install(
  DIRECTORY launch description
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_include_directories(include)
ament_export_libraries(remote_control)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  hardware_interface
  pluginlib
)

install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
  FILES_MATCHING PATTERN "*.xacro"
)


### --- Publisher Executable ---

# Add shared library for controller plugin
add_library(remote_control_publisher SHARED
  src/remote_control_publisher.cpp
)

target_include_directories(remote_control_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(remote_control_publisher
  rclcpp
  sensor_msgs
  std_msgs
  controller_interface
  hardware_interface
)

# Export plugin description
pluginlib_export_plugin_description_file(controller_interface
  remote_control_controller.xml
)

install(
  TARGETS remote_control_publisher
  EXPORT export_remote_control_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# Final package macro
ament_package()
