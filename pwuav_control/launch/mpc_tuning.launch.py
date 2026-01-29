from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pwuav_control',
            executable='mpc_controller_node',
            name='mpc_controller_node',
            output='screen'
        ),
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
            output='screen'
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=[
                'debug_velocity/vector/x',  # desired velocity
                'debug_velocity/vector/y',  # current velocity
                # 'debug_yaw_rate/vector/x',  # desired yaw rate
                # 'debug_yaw_rate/vector/y'   # current yaw rate
            ],
            output='screen'
        )
    ])
