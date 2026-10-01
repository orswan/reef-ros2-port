"""X3 vision run (P08): the stock truth-fed controller flies the vision
profile in front of the vision scene while an RGB-D camera renders
(worlds/x3_vision.sdf: x3_flight + Sensors (ogre2) + reef_vision_scene +
reef_x3_rgbd). Run through scripts/run_x3_scenario.sh --vision (isolation,
offline guards, monitoring, manifest, analysis).

The Gazebo server runs with -s --headless-rendering (EGL; software
rendering in these containers). Camera topics are bridged by their own
bridge (config/bridge_camera.yaml); raw images are not recorded (size),
camera_check writes the interface results, sample frames, and the
estimator's inputs from the ROS graph.

The REEF chain runs OPEN LOOP beside the stock controller: camera ->
reef_rgbd_odometry (REPLACEMENT odometry, not demo_rgbd) -> rgbd_to_velocity
(ported) -> REEF estimator (config/simulation_vision.yaml: RGB-D on, mocap
velocity off). Attitude for REEF is still the TRUTH attitude (x3_imu_adapter,
idealized); the REEF adapter (truth-derived velocity observations) is NOT
started, so no truth velocity topic exists in vision runs.

Arguments: output_dir:=<dir>, params_file:=<merged yaml>, headless:=true|false
(false adds a separate `gz sim -g` viewer), record:=true|false,
record_camera:=true|false (diagnostic, default false: also record the raw
RGB and depth frames for offline odometry replay; large).
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

TOPICS = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range', '/x3/cmd_vel', '/x3/scenario/phase',
          '/x3/camera/camera_info', '/x3/reef/cam_to_init', '/x3/reef/vo/health',
          '/x3/reef/rgbd_to_velocity/body_level_frame', '/x3/reef/rgbd_to_velocity/init_frame',
          '/x3/reef/imu/data', '/x3/reef/xyz_estimate', '/x3/reef/xyz_debug_estimate', '/x3/reef/is_flying_reef',
          '/x3/reef/diagnostics']
REQUIRED = ['/x3/reef/cam_to_init', '/x3/reef/rgbd_to_velocity/body_level_frame', '/x3/reef/imu/data',
            '/x3/reef/xyz_estimate']


def generate_launch_description():
    share = FindPackageShare('reef_sim')
    output_dir = LaunchConfiguration('output_dir')
    params_file = LaunchConfiguration('params_file')
    record = LaunchConfiguration('record')
    record_camera = LaunchConfiguration('record_camera')
    sim_time = {'use_sim_time': True}
    reef_config = PathJoinSubstitution([FindPackageShare('reef_estimator'), 'config'])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    world = PathJoinSubstitution([share, 'worlds', 'x3_vision.sdf'])
    runner = Node(
        package='reef_sim', executable='scenario_runner', name='scenario_runner', output='screen',
        parameters=[params_file, sim_time, {
            'result_file': PathJoinSubstitution([output_dir, 'scenario_result.json']),
            'required_extra': REQUIRED,
            'record_topics': PythonExpression([f'{TOPICS!r} if "', record, '" == "true" else [""]'])}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('record_camera', default_value='false'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([share, 'models'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -s --headless-rendering ', world], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g', 'on_exit_shutdown': 'false'}.items(),
            condition=UnlessCondition(LaunchConfiguration('headless'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='x3_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([share, 'config', 'bridge.yaml'])}, sim_time]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([share, 'config', 'bridge_camera.yaml'])}, sim_time]),
        Node(package='reef_sim', executable='imu_noise', name='imu_noise', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_sim', executable='range_sensor', name='range_sensor', output='screen',
             parameters=[params_file, sim_time]),
        Node(package='reef_sim', executable='camera_check', name='camera_check', output='screen',
             parameters=[params_file, sim_time, {'output_dir': output_dir}]),
        Node(package='reef_rgbd_odometry', executable='reef_rgbd_odometry', name='reef_rgbd_odometry',
             namespace='/x3/reef', output='screen', parameters=[params_file, sim_time],
             remappings=[('image', '/x3/camera/image'), ('depth', '/x3/camera/depth'),
                         ('camera_info', '/x3/camera/camera_info')]),
        Node(package='rgbd_to_velocity', executable='rgbd_to_velocity_node', name='rgbd_to_velocity_node',
             namespace='/x3/reef', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('rgbd_to_velocity'), 'config', 'x3_sim_camera.yaml']),
                         sim_time]),
        Node(package='reef_x3_adapter', executable='x3_imu_adapter', name='x3_imu_adapter', output='screen',
             parameters=[sim_time]),
        Node(package='reef_estimator', executable='reef_estimator_node', name='reef_estimator',
             namespace='/x3/reef', output='screen', remappings=[('sonar', '/x3/range')],
             parameters=[PathJoinSubstitution([reef_config, 'estimator_master.yaml']),
                         PathJoinSubstitution([reef_config, 'simulation.yaml']),
                         PathJoinSubstitution([reef_config, 'simulation_vision.yaml']), sim_time]),
        runner,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'bag']), '--topics', *TOPICS],
            name='recorder', output='screen', condition=IfCondition(record)),
        ExecuteProcess(   # diagnostic only (record_camera:=true): raw frames for offline replay
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '--disable-keyboard-controls', '-s', 'mcap',
                 '-o', PathJoinSubstitution([output_dir, 'camera_bag']), '--topics', '/x3/camera/image',
                 '/x3/camera/depth', '/x3/camera/camera_info'],
            name='camera_recorder', output='screen',
            condition=IfCondition(PythonExpression(['"', record, '" == "true" and "', record_camera, '" == "true"']))),
        RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='scenario finished'))])),
    ])
