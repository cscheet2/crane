from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  robot_description = {"robot_description": Command([
    PathJoinSubstitution([
      FindPackageShare("remote_control"),
      "description",
      "controller.urdf.xacro",
    ])
  ])}

  return LaunchDescription([
    Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
        robot_description,
        PathJoinSubstitution([
          FindPackageShare("remote_control"),
          "config",
          "controller.yaml",
        ]),
      ],
      output="both",
      emulate_tty=True,
      name="controller_manager",
    )
  ])