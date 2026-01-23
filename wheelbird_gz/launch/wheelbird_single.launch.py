import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    sys_autostart = LaunchConfiguration('sys_autostart').perform(context)
    model = LaunchConfiguration('model').perform(context)
    world = LaunchConfiguration('world').perform(context)
    instance_id = int(LaunchConfiguration('instance_id').perform(context))
    standalone = LaunchConfiguration('standalone').perform(context)
    
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    R = LaunchConfiguration('R').perform(context)
    P = LaunchConfiguration('P').perform(context)
    Y = LaunchConfiguration('Y').perform(context)
    
    udp_local_port = 14540 + instance_id
    udp_remote_port = 14580 + instance_id
    tgt_system_id = instance_id + 1
    
    fcu_url = f'udp://:{udp_local_port}@127.0.0.1:{udp_remote_port}'
    
    px4_env = {
        'PX4_SYS_AUTOSTART': sys_autostart,
        'PX4_GZ_MODEL_POSE': f'{x},{y},{z},{R},{P},{Y}',
        'PX4_SIM_MODEL': model,
        'PX4_GZ_WORLD': world,
        'PX4_GZ_MODEL': model
    }

    if standalone == '1':
        px4_env['PX4_GZ_STANDALONE'] = '1'

    px4_binary_path = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    
    px4_process = ExecuteProcess(
        cmd=[px4_binary_path, '-i', str(instance_id)],
        output='screen',
        additional_env=px4_env,
        shell=True
    )

    config_file_name = f'wheelbird{instance_id}_config.yaml'
    config_file_path = os.path.join('/home/ubuntu/px4_ws/src/wheelbird_gz', 'config', config_file_name)

    mavros_share = get_package_share_directory('mavros')
    px4_pluginlists = os.path.join(mavros_share, 'launch', 'px4_pluginlists.yaml')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            px4_pluginlists,
            config_file_path,
            {
                'fcu_url': fcu_url,
                'tgt_system': instance_id + 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            }
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    return [px4_process, mavros_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('HOME') + '/PX4-Autopilot', description='PX4 root directory'),
        DeclareLaunchArgument('sys_autostart', default_value='22001', description='sys number'),
        DeclareLaunchArgument('model', default_value='wheelbird', description='Model name (e.g. wheelbird, iris)'),
        DeclareLaunchArgument('world', default_value='obstacles_wall', description='World name'),
        DeclareLaunchArgument('instance_id', default_value='1', description='Instance ID (affects ports and sys_id)'),
        DeclareLaunchArgument('standalone', default_value='0', description='Run simulation without GUI'),

        DeclareLaunchArgument('x', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('z', default_value='0.3', description='Initial Z position'),
        DeclareLaunchArgument('R', default_value='0.0', description='Initial Roll'),
        DeclareLaunchArgument('P', default_value='0.0', description='Initial Pitch'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Initial Yaw'),

        OpaqueFunction(function=launch_setup)
    ])