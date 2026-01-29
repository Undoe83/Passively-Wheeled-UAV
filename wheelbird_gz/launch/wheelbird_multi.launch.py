import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    single_launch_path = PathJoinSubstitution([
        get_package_share_directory('wheelbird_gz'),
        'launch',
        'wheelbird_single.launch.py'
    ])

    px4_dir = os.environ.get('HOME') + '/PX4-Autopilot'

    # ==========================================================
    #                       wheelbird1
    # ==========================================================
    wheelbird1 = GroupAction([
        PushRosNamespace('wheelbird1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_launch_path),
            launch_arguments={
                'px4_dir': px4_dir,
                'model': 'wheelbird',
                'instance_id': '1',  # 포트: 14541/14581, SysID: 2
                'x': '2.0',
                'y': '3.6',
                'z': '0.3',
                'R': '0.0', 'P': '0.0', 'Y': '0.0'
            }.items()
        )
    ])

    tf_wheelbird1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['2.0', '3.6', '0.3', '0', '0', '0', 'map', 'wheelbird1/odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ==========================================================
    #                       wheelbird2
    # ==========================================================
    # wheelbird2 = TimerAction(
    #     period=10.0,
    #     actions=[
    #         GroupAction([
    #             PushRosNamespace('wheelbird2'),
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(single_launch_path),
    #                 launch_arguments={
    #                     'px4_dir': px4_dir,
    #                     'standalone': '1',
    #                     'model': 'wheelbird',
    #                     'instance_id': '2',  # 포트: 14542/14582, SysID: 3
    #                     'x': '2.0',          # 옆으로 2m 이동해서 스폰
    #                     'y': '1.2',
    #                     'z': '0.3',
    #                     'R': '0.0', 'P': '0.0', 'Y': '0.0'
    #                 }.items()
    #             )
    #         ])
    #     ]
    # )

    # tf_wheelbird2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['2.0', '1.2', '0.3', '0', '0', '0', 'map', 'wheelbird2/odom'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # # ==========================================================
    # #                       wheelbird3
    # # ==========================================================
    # wheelbird3 = TimerAction(
    #     period=15.0,
    #     actions=[
    #         GroupAction([
    #             PushRosNamespace('wheelbird3'),
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(single_launch_path),
    #                 launch_arguments={
    #                     'px4_dir': px4_dir,
    #                     'standalone': '1',
    #                     'model': 'wheelbird',
    #                     'instance_id': '3',
    #                     'x': '2.0',
    #                     'y': '-1.2',
    #                     'z': '0.3',
    #                     'R': '0.0', 'P': '0.0', 'Y': '0.0'
    #                 }.items()
    #             )
    #         ])
    #     ]
    # )

    # tf_wheelbird3 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['2.0', '-1.2', '0.3', '0', '0', '0', 'map', 'wheelbird3/odom'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # # ==========================================================
    # #                       wheelbird4
    # # ==========================================================
    # wheelbird4 = TimerAction(
    #     period=20.0,
    #     actions=[
    #         GroupAction([
    #             PushRosNamespace('wheelbird4'),
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(single_launch_path),
    #                 launch_arguments={
    #                     'px4_dir': px4_dir,
    #                     'standalone': '1',
    #                     'model': 'wheelbird',
    #                     'instance_id': '4',
    #                     'x': '2.0',
    #                     'y': '-3.6',
    #                     'z': '0.3',
    #                     'R': '0.0', 'P': '0.0', 'Y': '0.0'
    #                 }.items()
    #             )
    #         ])
    #     ]
    # )

    # tf_wheelbird4 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['2.0', '-3.6', '0.3', '0', '0', '0', 'map', 'wheelbird4/odom'],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    rviz_config_path = PathJoinSubstitution([
        get_package_share_directory('wheelbird_gz'),
        'rviz',
        'wheelbird.rviz'
    ])

    octomap_node = Node(
        package='octomap_generator',
        executable='octomap_publish_node',
        name='octomap_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        wheelbird1,
        # wheelbird2,
        # wheelbird3,
        # wheelbird4,
        tf_wheelbird1,
        # tf_wheelbird2,
        # tf_wheelbird3,
        # tf_wheelbird4,
        octomap_node,
        rviz_node
    ])