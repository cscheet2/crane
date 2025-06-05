from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to controller config YAML
    controller_config = PathJoinSubstitution([
        FindPackageShare("controller"),
        "config",
        "controller.yaml",
    ])

    # Load the controller manager with your joystick plugin
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
        arguments=[],
    )

    # Spawner node for joint_state_broadcaster (publishes joystick state)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_spawner,
    ])
