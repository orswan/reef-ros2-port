"""Run a simple Gazebo Harmonic world and bridge its clock into ROS 2.

Run through scripts/run_clock_demo.sh so the environment is explicit.

Arguments:
  world:=<path to .sdf>   default: sim/worlds/clock_demo.sdf
  headless:=true|false    true runs the server only (no GUI window)
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_WORLD = str(Path(__file__).resolve().parents[1] / 'worlds' / 'clock_demo.sdf')


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    def gz_sim(extra_flags, condition):
        # -r: start running immediately so /clock advances without pressing play.
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-r ', extra_flags, world],
                              'on_exit_shutdown': 'true'}.items(),
            condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD),
        DeclareLaunchArgument('headless', default_value='false'),
        gz_sim('', UnlessCondition(headless)),
        gz_sim('-s ', IfCondition(headless)),
        # Gazebo -> ROS only ('[' direction). /clock is a gz.msgs.Clock topic.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
    ])
