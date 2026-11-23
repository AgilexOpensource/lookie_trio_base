import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'lookie_trio_base'


def _launch_setup(context, *args, **kwargs):
    overrides = {}
    for name in ('port_name', 'odom_frame', 'base_frame'):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    value = LaunchConfiguration('publish_odom_tf').perform(context).strip()
    if value:
        overrides['publish_odom_tf'] = value == 'true'

    parameters = [LaunchConfiguration('params_file').perform(context)]
    if overrides:
        parameters.append(overrides)
    return [
        Node(
            package=PACKAGE,
            executable=f'{PACKAGE}_node',
            name=PACKAGE,
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(get_package_share_directory(PACKAGE), 'config', f'{PACKAGE}.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Node namespace; topics are relative to it.'),
        DeclareLaunchArgument('params_file', default_value=default_params, description=f'{PACKAGE} parameter file.'),
        DeclareLaunchArgument('port_name', default_value='', description='CAN interface (empty = use params_file).'),
        DeclareLaunchArgument('odom_frame', default_value='', description='Odometry frame (empty = use params_file).'),
        DeclareLaunchArgument('base_frame', default_value='', description='Base frame (empty = use params_file).'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='', choices=['', 'true', 'false'],
            description='Broadcast odom_frame -> base_frame TF (empty = use params_file).'),
        OpaqueFunction(function=_launch_setup),
    ])
