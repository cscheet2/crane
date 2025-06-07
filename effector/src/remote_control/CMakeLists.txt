cmake_minimum_required(VERSION 3.8)
project(remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED IMPORTED_TARGET libevdev)

# add library
add_library(remote_control SHARED
  src/remote_control.cpp
)

# add include dirs
target_include_directories(remote_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBEVDEV_INCLUDE_DIRS}
)

target_link_libraries(remote_control
  PkgConfig::LIBEVDEV
)

# link dependencies
ament_target_dependencies(remote_control
  rclcpp
  sensor_msgs
  std_msgs
  hardware_interface
  pluginlib
)

# pluginlib export
pluginlib_export_plugin_description_file(hardware_interface 
  remote_control.xml
)

# install headers and library
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS remote_control
  EXPORT export_remote_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch description
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(remote_control)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  hardware_interface
  pluginlib
)

target_link_libraries(remote_control
  evdev
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
