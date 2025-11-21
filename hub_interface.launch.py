from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hub_name', default_value='Pybricks Hub'),
        
        Node(
            package='lego_hub_interface',
            executable='hub_node',
            name='lego_hub_node',
            output='screen',
            parameters=[{'hub_name': LaunchConfiguration('hub_name')}],
        ),
    ])