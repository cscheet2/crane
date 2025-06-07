from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    # Path to your main xacro file
    xacro_file = os.path.join(
        get_package_share_directory('remote_control'),
        'description',
        'remote_control.urdf.xacro'
    )

    # Use subprocess to run xacro and get expanded URDF XML as string
    robot_description_raw = subprocess.check_output(['xacro', xacro_file]).decode()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_raw}],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[{'robot_description': robot_description_raw}],
            output='screen'
        ),
    ])
