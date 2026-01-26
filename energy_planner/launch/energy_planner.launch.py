import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('energy_planner')
    config_file = os.path.join(pkg_dir, 'config', 'energy_aware_path_planner.yaml')

    return LaunchDescription([
        Node(
            package='energy_planner',
            executable='energy_aware_path_planner',
            name='energy_path_planner',
            output='screen',
            parameters=[config_file]
        )
    ])
