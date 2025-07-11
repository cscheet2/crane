cmake_minimum_required(VERSION 3.8)
project(hopper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)

# add_executable(drivetrain src/drivetrain.cpp)
# ament_target_dependencies(drivetrain rclcpp interfaces)

# Action Server
add_library(action_server SHARED
  src/linear_actuator_action_server.cpp
)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_server 
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_server
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(action_server 
  PLUGIN "linear_actuator_action::LinearActuatorActionServer"
  EXECUTABLE linear_actuator_action_server
)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Action Client
add_library(action_client SHARED
  src/linear_actuator_action_client.cpp
)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client 
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_client
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(action_client 
  PLUGIN "linear_actuator_action::LinearActuatorActionClient"
  EXECUTABLE linear_actuator_action_client
)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line -kips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
