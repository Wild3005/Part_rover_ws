from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path ke config YAML
    # package_share = FindPackageShare('rover_middleware').find('rover_middleware')
    package_share = os.path.join(
        os.getenv('HOME'), 'Documents', 'Rover_ws', 'src', 'rover_middleware')
    config_path = os.path.join(package_share, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='rover_middleware',
            executable='arm_rover',
            name='arm_pov',
            output='screen',
            parameters=[config_path]
        ),
    ])