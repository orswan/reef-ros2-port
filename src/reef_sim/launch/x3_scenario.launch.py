"""X3 scenario: Gazebo world, ROS bridge, idealized range, scenario, recorder.

Run it through scripts/run_x3_scenario.sh, which sets up the environment
(assets on GZ_SIM_RESOURCE_PATH, per-run isolation, no Fuel access), writes
the manifest, monitors the processes, and analyzes the bag.

Arguments:
  output_dir:=<dir>     required; bag in <dir>/bag, result in <dir>/scenario_result.json
  params_file:=<yaml>   scenario/sensor parameters (default: config/x3_scenario.yaml)
  headless:=true|false  false also starts the Gazebo GUI on $DISPLAY, as a
                        separate viewer process (`gz sim -g`). The server always
                        runs with `-s`: in combined mode, gz-sim 8 makes the
                        server wait for a world-path handshake from the GUI
                        (wait_gui), and a missed handshake stalls the simulation.
  record:=true|false    rosbag2 recording
  enable_range:=true|false   test hook: false omits the range stream
  with_estimator:=true|false also run the REEF adapter and the ported REEF
                        estimator (combined filter; namespace /x3/reef) beside
                        the truth-fed stock controller, and record their topics.
                        REEF is NOT in the control loop.

The launch shuts down when the scenario runner exits. The recorder then gets
SIGINT and finalizes the bag.
"""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RECORDED_TOPICS = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range',
                   '/x3/cmd_vel', '/x3/scenario/phase']
REEF_TOPICS = ['/x3/reef/imu/data', '/x3/reef/sonar', '/x3/reef/mocap_velocity/body_level_frame',
               '/x3/reef/xyz_estimate', '/x3/reef/xyz_debug_estimate', '/x3/reef/is_flying_reef',
               '/x3/reef/input_labels', '/x3/reef/diagnostics']
REEF_REQUIRED = ['/x3/reef/imu/data', '/x3/reef/mocap_velocity/body_level_frame', '/x3/reef/xyz_estimate']


def generate_launch_description():
    share = FindPackageShare('reef_sim')
    output_dir = LaunchConfiguration('output_dir')
    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')
    record = LaunchConfiguration('record')
    enable_range = LaunchConfiguration('enable_range')
    with_estimator = LaunchConfiguration('with_estimator')
    estimator_on = PythonExpression(['"', with_estimator, '" == "true"'])
    reef_config = PathJoinSubstitution([FindPackageShare('reef_estimator'), 'config'])
    world = PathJoinSubstitution([share, 'worlds', 'x3_flight.sdf'])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    sim_time = {'use_sim_time': True}


    runner = Node(
        package='reef_sim', executable='scenario_runner', name='scenario_runner',
        output='screen',
        parameters=[params_file, sim_time, {
            'result_file': PathJoinSubstitution([output_dir, 'scenario_result.json']),
            'require_range': PythonExpression(["'", enable_range, "' == 'true'"]),
            'record_topics': PythonExpression(
                [f'({RECORDED_TOPICS!r} + ({REEF_TOPICS!r} if "', with_estimator,
                 '" == "true" else [])) if "', record, '" == "true" else [""]']),
            'required_extra': PythonExpression(
                [f'{REEF_REQUIRED!r} if "', with_estimator, '" == "true" else [""]']),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([share, 'config', 'x3_scenario.yaml'])),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('enable_range', default_value='true'),
        DeclareLaunchArgument('with_estimator', default_value='false'),
        # reef_x3 model; the upstream X3 meshes (assets/models) are added by the run script.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([share, 'models'])),
        # Server: always headless (no GUI handshake); its exit ends the run.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -s ', world], 'on_exit_shutdown': 'true'}.items()),
        # Optional viewer: connects to the running server; closing it does not stop the run.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g', 'on_exit_shutdown': 'false'}.items(),
            condition=UnlessCondition(headless)),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='x3_bridge',
             output='screen', parameters=[
                 {'config_file': PathJoinSubstitution([share, 'config', 'bridge.yaml'])}, sim_time]),
        Node(package='reef_sim', executable='imu_noise', name='imu_noise', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_sim', executable='range_sensor', name='range_sensor', output='screen',
             parameters=[params_file, sim_time], condition=IfCondition(enable_range)),
        # REEF: adapter (IDEALIZED attitude from truth) and the ported estimator.
        Node(package='reef_sim', executable='reef_adapter', name='reef_adapter', output='screen',
             parameters=[sim_time], condition=IfCondition(estimator_on)),
        Node(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
             namespace='/x3/reef', output='screen', condition=IfCondition(estimator_on),
             parameters=[PathJoinSubstitution([reef_config, 'estimator_master.yaml']),
                         PathJoinSubstitution([reef_config, 'simulation.yaml']), sim_time]),
        runner,
        # Keyboard controls off: a recorder reading a terminal from the
        # background is stopped by SIGTTIN (seen with `docker compose exec`).
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'bag']), '--topics', *RECORDED_TOPICS],
            name='recorder', output='screen',
            condition=IfCondition(PythonExpression(['"', record, '" == "true" and not ', estimator_on]))),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'bag']), '--topics', *RECORDED_TOPICS, *REEF_TOPICS],
            name='recorder', output='screen',
            condition=IfCondition(PythonExpression(['"', record, '" == "true" and ', estimator_on]))),
        RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='scenario finished'))])),
    ])

