"""Replay an X3 recording through the REEF adapter and estimator (ROS graph).

Run it through scripts/replay_reef_estimator.sh, which owns the session,
chooses an unused ROS domain, and checks that the bag is the only /clock
source. Arguments:
  bag:=<run>/bag        recording to play (its recorded /clock drives time)
  output_dir:=<dir>     new bag with the replayed inputs and REEF outputs
  rate:=1.0             playback rate

Only the simulation inputs are played (/clock, /x3/truth/odom, /x3/imu,
/x3/range, /x3/scenario/phase): never /x3/cmd_vel, and never REEF outputs
recorded by a live run. Playback starts after a delay so the estimator and
the recorder are subscribed first. The launch shuts down when the player
exits. IDEALIZED INPUTS as in the live run (attitude from truth).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PLAYED = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range', '/x3/scenario/phase']
REEF_INPUTS = ['/x3/reef/imu/data', '/x3/reef/sonar', '/x3/reef/mocap_velocity/body_level_frame']
RECORDED = PLAYED[1:] + REEF_INPUTS + ['/x3/reef/xyz_estimate', '/x3/reef/xyz_debug_estimate',
                                       '/x3/reef/is_flying_reef', '/x3/reef/input_labels', '/x3/reef/diagnostics']


def generate_launch_description():
    reef_config = PathJoinSubstitution([FindPackageShare('reef_estimator'), 'config'])
    sim_time = {'use_sim_time': True}
    player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--disable-keyboard-controls',
             '--delay', '3.0', '--rate', LaunchConfiguration('rate'), '--topics', *PLAYED],
        name='player', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        Node(package='reef_sim', executable='reef_adapter', name='reef_adapter', output='screen',
             parameters=[sim_time]),
        Node(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
             namespace='/x3/reef', output='screen',
             parameters=[PathJoinSubstitution([reef_config, 'estimator_master.yaml']),
                         PathJoinSubstitution([reef_config, 'simulation.yaml']), sim_time]),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([LaunchConfiguration('output_dir'), 'bag']),
                 '--topics', *RECORDED],
            name='recorder', output='screen'),
        player,
        RegisterEventHandler(OnProcessExit(
            target_action=player, on_exit=[EmitEvent(event=Shutdown(reason='replay finished'))])),
    ])
