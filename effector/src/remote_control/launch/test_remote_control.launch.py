from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
import os

import xacro  # import xacro python module

def generate_launch_description():
    urdf_xacro_path = os.path.join(
        get_package_share_directory('remote_control'),
        'description',
        'remote_control.urdf.xacro'
    )

    # Process the xacro file to generate URDF XML string
    doc = xacro.process_file(urdf_xacro_path)
    robot_description_xml = doc.toxml()

    robot_description = {'robot_description': robot_description_xml}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description],
            output='screen'
        ),
        # Optionally publish robot_description parameter explicitly
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/controller_manager', 'robot_description', robot_description_xml],
            shell=True
        ),
    ])
