from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('gamepad'),
        'urdf',
        'gamepad.urdf.xacro'
    )
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('gamepad'),
        'config',
        'gamepad.yaml'
    )
    
    # Process the URDF file
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )
    
    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, config_file],
        output='screen',
    )
    
    # Spawner node
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gamepad_controller', '--controller-manager-timeout', '30'],
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        controller_spawner,
    ])