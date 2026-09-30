"""Start the REEF estimator node (vertical filter; horizontal not ported).

    ros2 launch reef_estimator reef_estimator.launch.py \
        [params_file:=...] [overrides_file:=...] [use_sim_time:=true|false] [namespace:=...]

params_file defaults to config/estimator_master.yaml (the shipped hardware
values). For simulation pass overrides_file:=<share>/config/simulation.yaml,
which disables the RC switch. Topic names and QoS: docs/INTERFACES.md
section 3. Invalid parameters make the node exit with status 1.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('reef_estimator')) / 'config'
    params = LaunchConfiguration('params_file')
    overrides = LaunchConfiguration('overrides_file')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    has_overrides = PythonExpression(["'", overrides, "' != ''"])
    common = dict(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
                  namespace=LaunchConfiguration('namespace'), output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(config / 'estimator_master.yaml')),
        DeclareLaunchArgument('overrides_file', default_value='',
                              description='optional second parameter file (for example config/simulation.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(**common, parameters=[params, overrides, sim_time], condition=IfCondition(has_overrides)),
        Node(**common, parameters=[params, sim_time], condition=UnlessCondition(has_overrides)),
    ])
