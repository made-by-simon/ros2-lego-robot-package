"""Launch file for LEGO Hub ROS 2 interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hub_name",
                default_value="Pybricks Hub",
                description="Bluetooth name of the LEGO hub",
            ),
            Node(
                package="lego_hub_ros2",
                executable="hub_node",
                name="lego_hub_node",
                output="screen",
                parameters=[{"hub_name": LaunchConfiguration("hub_name")}],
            ),
        ]
    )
