"""reef_control (P06): the ported controller and the DRY-RUN command sink.

    ros2 launch reef_control reef_control.launch.py [params_file:=...] [trace_file:=...] [sink:=true]

No hardware output exists: the sink records commands with the firmware's
interpretation and refuses hardware:=true (NOT IMPLEMENTED, P10).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('reef_control')) / 'config' / 'reef_control_quad.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('trace_file', default_value=''),
        DeclareLaunchArgument('sink', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='reef_control', executable='reef_control_node', name='reef_control_pid', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='reef_control', executable='reef_control_sink', name='reef_control_sink', output='screen',
             condition=IfCondition(LaunchConfiguration('sink')),
             parameters=[{'trace_file': LaunchConfiguration('trace_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
