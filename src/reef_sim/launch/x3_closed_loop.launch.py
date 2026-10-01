"""REEF-controlled X3 (P07): Gazebo world without the stock controller, REEF
estimator and REEF controller in the loop, stand-in low-level loop
(reef_fc_standin, DEVELOPMENT TOOL, not ROSflight) on the motor model.

Run it through scripts/run_x3_scenario.sh --closed-loop (isolation,
offline guards, monitoring, manifest, analysis).

    estimator inputs (IDEALIZED, as in P05): truth attitude, idealized range,
    simulated velocity observations; IMU vibration assumption
    /x3/reef/xyz_estimate -> reef_control (/x3/reef) -> /x3/reef/command
      -> reef_fc_standin (truth attitude + noise-free gyro) -> /x3/fc/motor_speed
      -> MulticopterMotorModel (Gazebo, the only physics)

Arguments: output_dir:=<dir> (required), params_file:=<merged yaml>,
headless:=true|false (false adds a separate `gz sim -g` viewer),
record:=true|false, control_params:=<reef_control yaml> (default
reef_control_x3_sim.yaml: the shipped quad gains with dI = 0, see that file),
control_respawn:=true|false (P07b controller-restart case only; default false
so that a controller crash in any other run is never masked),
vision:=true|false (P08: the closed loop on vision; default false).
With vision:=true the world is x3_closed_loop_vision.sdf (RGB-D camera, vision
scene, run with --headless-rendering), the camera chain runs (camera bridge,
camera_check, reef_rgbd_odometry: a REPLACEMENT odometry, rgbd_to_velocity),
the estimator adds config/simulation_vision.yaml (RGB-D on, mocap velocity
off), and the REEF adapter (truth-derived velocity observations) is NOT
started: REEF's only horizontal velocity input is vision.
The merged params_file also reaches the nodes with P07b test hooks and the
controller (key /x3/reef/reef_control_pid), so a scenario overlay can enable
hooks or the idealized mocap pose; all are off by default.
The launch shuts down when the scenario runner exits.
"""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TOPICS = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range', '/x3/scenario/phase',
          '/x3/reef/imu/data', '/x3/reef/mocap_velocity/body_level_frame', '/x3/reef/xyz_estimate',
          '/x3/reef/xyz_debug_estimate', '/x3/reef/is_flying_reef', '/x3/reef/input_labels',
          '/x3/reef/diagnostics', '/x3/reef/desired_state', '/x3/reef/controller_state',
          '/x3/reef/command', '/x3/reef/status', '/x3/fc/arm', '/x3/fc/motor_speed', '/x3/fc/debug',
          '/x3/fc/label', '/x3/reef/pose_stamped', '/x3/test/fault']
VISION_TOPICS = ['/x3/camera/camera_info', '/x3/reef/cam_to_init', '/x3/reef/vo/health',
                 '/x3/reef/rgbd_to_velocity/body_level_frame', '/x3/reef/rgbd_to_velocity/init_frame']


def generate_launch_description():
    share = FindPackageShare('reef_sim')
    output_dir = LaunchConfiguration('output_dir')
    params_file = LaunchConfiguration('params_file')
    record = LaunchConfiguration('record')
    sim_time = {'use_sim_time': True}
    reef_config = PathJoinSubstitution([FindPackageShare('reef_estimator'), 'config'])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    vision = LaunchConfiguration('vision')
    vision_on = IfCondition(vision)
    world = PathJoinSubstitution([share, 'worlds', PythonExpression(
        ["'x3_closed_loop_vision.sdf' if '", vision, "' == 'true' else 'x3_closed_loop.sdf'"])])
    gz_server = PythonExpression(["'-r -s --headless-rendering ' if '", vision, "' == 'true' else '-r -s '"])
    estimator_params = [PathJoinSubstitution([reef_config, 'estimator_master.yaml']),
                        PathJoinSubstitution([reef_config, 'simulation.yaml'])]
    runner = Node(
        package='reef_sim', executable='closed_loop_runner', name='closed_loop_runner', output='screen',
        parameters=[params_file, sim_time, {
            'result_file': PathJoinSubstitution([output_dir, 'scenario_result.json']),
            'record_topics': PythonExpression(
                [f'({TOPICS!r} + ({VISION_TOPICS!r} if "', vision, '" == "true" else [])) if "', record,
                 '" == "true" else [""]'])}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('control_respawn', default_value='false'),
        DeclareLaunchArgument('vision', default_value='false'),
        DeclareLaunchArgument('control_params', default_value=PathJoinSubstitution(
            [FindPackageShare('reef_control'), 'config', 'reef_control_x3_sim.yaml'])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([share, 'models'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [gz_server, world], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g', 'on_exit_shutdown': 'false'}.items(),
            condition=UnlessCondition(LaunchConfiguration('headless'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='x3_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([share, 'config', 'bridge_closed_loop.yaml'])},
                         sim_time]),
        Node(package='reef_sim', executable='imu_noise', name='imu_noise', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_sim', executable='range_sensor', name='range_sensor', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_x3_adapter', executable='x3_imu_adapter', name='x3_imu_adapter', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_sim', executable='reef_adapter', name='reef_adapter', output='screen',
             parameters=[params_file, sim_time], condition=UnlessCondition(vision)),
        Node(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
             namespace='/x3/reef', output='screen', remappings=[('sonar', '/x3/range')],
             parameters=[*estimator_params, sim_time], condition=UnlessCondition(vision)),
        # P08 vision chain (vision:=true): camera -> REPLACEMENT odometry -> rgbd_to_velocity -> REEF.
        Node(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
             namespace='/x3/reef', output='screen', remappings=[('sonar', '/x3/range')],
             parameters=[*estimator_params, PathJoinSubstitution([reef_config, 'simulation_vision.yaml']),
                         sim_time], condition=vision_on),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([share, 'config', 'bridge_camera.yaml'])}, sim_time],
             condition=vision_on),
        Node(package='reef_sim', executable='camera_check', name='camera_check', output='screen',
             parameters=[params_file, sim_time, {'output_dir': output_dir}], condition=vision_on),
        Node(package='reef_rgbd_odometry', executable='reef_rgbd_odometry', name='reef_rgbd_odometry',
             namespace='/x3/reef', output='screen', parameters=[params_file, sim_time],
             remappings=[('image', '/x3/camera/image'), ('depth', '/x3/camera/depth'),
                         ('camera_info', '/x3/camera/camera_info')], condition=vision_on),
        Node(package='rgbd_to_velocity', executable='rgbd_to_velocity_node', name='rgbd_to_velocity_node',
             namespace='/x3/reef', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('rgbd_to_velocity'), 'config', 'x3_sim_camera.yaml']),
                         sim_time], condition=vision_on),
        # REEF controller: reads only xyz_estimate and desired_state (and status, is_flying).
        Node(package='reef_control', executable='reef_control_node', name='reef_control_pid',
             namespace='/x3/reef', output='screen', respawn=LaunchConfiguration('control_respawn'),
             respawn_delay=0.5, parameters=[LaunchConfiguration('control_params'), params_file, sim_time]),
        # STAND-IN low-level loop (development tool).
        Node(package='reef_fc_standin', executable='reef_fc_standin', name='reef_fc_standin', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('reef_fc_standin'), 'config', 'x3_standin.yaml']),
                         params_file, sim_time],
             remappings=[('command', '/x3/reef/command'), ('arm', '/x3/fc/arm'),
                         ('gyro', '/x3/sim/imu_noise_free'), ('attitude', '/x3/fc/truth_odom'),
                         ('motor_speed', '/x3/fc/motor_speed'), ('debug', '/x3/fc/debug'),
                         ('status', '/x3/reef/status'), ('label', '/x3/fc/label')]),
        runner,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'bag']), '--topics', *TOPICS],
            name='recorder', output='screen',
            condition=IfCondition(PythonExpression(['"', record, '" == "true" and "', vision, '" != "true"']))),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'bag']), '--topics', *TOPICS, *VISION_TOPICS],
            name='recorder', output='screen',
            condition=IfCondition(PythonExpression(['"', record, '" == "true" and "', vision, '" == "true"']))),
        RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='scenario finished'))])),
    ])
