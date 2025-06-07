from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ros2_control_node = Node(
    package='ros2_control_node',
    executable='ros2_control_node',
    parameters=['gamepad_hardware.yaml'],
    output='screen'
  )

  joy_node = Node(
    package='joy',
    executable='joy_node',
    output='screen'
  )

  return LaunchDescription([
    ros2_control_node,
    joy_node,
  ])