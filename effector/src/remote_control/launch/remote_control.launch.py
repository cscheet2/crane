import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    pkg_share = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(pkg_share, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([
                FindPackageShare("remote_control"),
                "description",
                "remote_control.urdf.xacro",
            ])
        ])
    }

    controller_params = load_yaml('remote_control', 'config/remote_control.yaml')

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        emulate_tty=True,
        name="controller_manager",
        parameters=[robot_description, controller_params],
    )

    joystick_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joystick_publisher_controller"],
        output="screen"
    )

    return LaunchDescription([
        controller_manager_node,
        joystick_controller_spawner,
    ])
