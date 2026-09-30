"""The numerical helpers build and run without ROS.

Compiles src/dynamics.cpp and test/ros_free_check.cpp in a clean environment
(no ROS variables), with only include/ and Eigen on the include path, and runs
the result.
"""
import os
import subprocess
from pathlib import Path

import pytest

PKG = Path(os.environ.get('REEF_PKG_SOURCE', Path(__file__).resolve().parents[1]))
CXX = os.environ.get('REEF_CXX', 'g++')
EIGEN = os.environ.get('REEF_EIGEN_INCLUDE', '/usr/include/eigen3').split(';')[0]
CLEAN_ENV = {'PATH': '/usr/bin:/bin', 'LANG': 'C'}


def test_environment_has_no_ros():
    assert not any(k.startswith(('ROS_', 'AMENT_', 'COLCON')) for k in CLEAN_ENV)
    assert '/opt/ros' not in EIGEN


def test_helpers_compile_and_run_without_ros(tmp_path):
    exe = tmp_path / 'ros_free_check'
    cmd = [CXX, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-ffp-contract=off',
           '-I', str(PKG / 'include'), '-isystem', EIGEN,
           str(PKG / 'src' / 'dynamics.cpp'), str(PKG / 'test' / 'ros_free_check.cpp'),
           '-o', str(exe)]
    build = subprocess.run(cmd, env=CLEAN_ENV, capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], env=CLEAN_ENV, capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    assert 'ros_free_check: 0 failures' in run.stdout


def test_including_ros_would_fail(tmp_path):
    """Negative control: the same clean compile of a ROS header must fail."""
    src = tmp_path / 'uses_ros.cpp'
    src.write_text('#include <rclcpp/rclcpp.hpp>\nint main() { return 0; }\n')
    build = subprocess.run([CXX, '-std=c++17', '-fsyntax-only', '-I', str(PKG / 'include'),
                            '-isystem', EIGEN, str(src)],
                           env=CLEAN_ENV, capture_output=True, text=True)
    if build.returncode == 0:
        pytest.fail('rclcpp headers are reachable without ROS include paths; the check proves nothing')
