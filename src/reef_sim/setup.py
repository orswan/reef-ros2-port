from glob import glob

from setuptools import setup

package_name = 'reef_sim'


def tree(pattern):
    return [f for f in glob(pattern, recursive=True) if not f.endswith('/')]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/closed_loop', glob('config/closed_loop/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/reef_x3', glob('models/reef_x3/*')),
        ('share/' + package_name + '/models/reef_x3_rgbd', glob('models/reef_x3_rgbd/*')),
        ('share/' + package_name + '/models/reef_vision_scene', glob('models/reef_vision_scene/*.*')),
        ('share/' + package_name + '/models/reef_vision_scene/materials/textures',
         glob('models/reef_vision_scene/materials/textures/*.png')),
        ('share/' + package_name + '/assets', glob('assets/*.json')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='orswan',
    maintainer_email='orswan@stanford.edu',
    description='X3 quadrotor Gazebo scenario and data tools for the REEF estimator port',
    license='MIT',
    entry_points={
        'console_scripts': [
            'range_sensor = reef_sim.range_sensor:main',
            'imu_noise = reef_sim.imu_noise:main',
            'scenario_runner = reef_sim.scenario_runner:main',
            'closed_loop_runner = reef_sim.closed_loop_runner:main',
            'camera_check = reef_sim.camera_check:main',
            'analyze_vision = reef_sim.analyze_vision:main',
            'analyze_closed_loop = reef_sim.analyze_closed_loop:main',
            'analyze_x3_bag = reef_sim.analyze:main',
            'reef_adapter = reef_sim.reef_adapter:main',
            'x3_reef_offline = reef_sim.reef_offline:main',
            'analyze_reef_vertical = reef_sim.analyze_reef:main',
        ],
    },
)
