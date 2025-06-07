from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            'config/gamepad_hardware.yaml',
            'config/gamepad_controller.yaml',
        ],
        output='screen',
    )

    # Delay spawner start by 2 seconds to let ros2_control_node initialize
    controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gamepad_controller'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        ros2_control_node,
        controller_spawner,
    ])
