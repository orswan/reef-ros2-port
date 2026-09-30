"""Legacy parameter files under ROS 2, and the converted configuration.

- The verbatim master files (test/data/legacy_params) are what e4179f48
  shipped (Git blob IDs).
- Characterization: ROS 2 cannot load them as they are, and still rejects the
  matrix files after adding the ROS 2 wrapper, because they mix integers and
  floats in one list.
- config/estimator_master.yaml holds exactly the same keys and numeric values
  and loads under ROS 2 with list values as double arrays.
- config/simulation.yaml disables the RC switch explicitly.
- rosflight_msgs/RCRaw (vendored upstream v2.0.1) has the legacy layout.
"""
import hashlib
from pathlib import Path

import pytest
import rclpy
import rclpy.context
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosidl_runtime_py.utilities import get_message

PKG = Path(__file__).resolve().parents[1]
LEGACY = PKG / 'test' / 'data' / 'legacy_params'
MASTER = PKG / 'config' / 'estimator_master.yaml'
SIM = PKG / 'config' / 'simulation.yaml'
BLOBS = {'xy_est_params.yaml': '6d113d3c848f5fc4c5bcdf7f8b84b736c9671dc5',
         'z_est_params.yaml': '2ab5bf6bb3ae6590374d54f93f9b78f2947d5bb0',
         'basic_params.yaml': '7aece7de76b94f33758309d0712da0bf19432690'}


def git_blob(path):
    data = path.read_bytes()
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def load_with_rcl(*files):
    """Parameters a node receives from params files, or raise the rcl error."""
    ctx = rclpy.context.Context()
    args = ['--ros-args']
    for f in files:
        args += ['--params-file', str(f)]
    rclpy.init(context=ctx, args=args)
    try:
        node = Node('reef_estimator', context=ctx, allow_undeclared_parameters=True,
                    automatically_declare_parameters_from_overrides=True)
        params = {name: node.get_parameter(name) for name in node._parameters}
        node.destroy_node()
        return params
    finally:
        rclpy.shutdown(context=ctx)


def wrapped(tmp_path, name):
    """The verbatim file, indented under the ROS 2 `/**: ros__parameters:` keys."""
    out = tmp_path / name
    body = (LEGACY / name).read_text().splitlines()
    out.write_text('/**:\n  ros__parameters:\n' + ''.join(f'    {line}\n' for line in body))
    return out


def legacy_values():
    merged = {}
    for name in BLOBS:
        merged.update(yaml.safe_load((LEGACY / name).read_text()))
    return merged


@pytest.mark.parametrize('name', sorted(BLOBS))
def test_verbatim_copies_are_the_shipped_files(name):
    assert git_blob(LEGACY / name) == BLOBS[name]


@pytest.mark.parametrize('name,reason', [
    ('xy_est_params.yaml', 'Sequences can only be values and not keys'),
    ('z_est_params.yaml', 'Sequences can only be values and not keys'),
    ('basic_params.yaml', 'Cannot have a value before ros__parameters'),
])
def test_verbatim_files_are_rejected_by_ros2(name, reason):
    with pytest.raises(Exception, match=reason):
        load_with_rcl(LEGACY / name)


@pytest.mark.parametrize('name', ['xy_est_params.yaml', 'z_est_params.yaml'])
def test_wrapped_matrix_files_are_rejected_for_mixed_lists(tmp_path, name):
    with pytest.raises(Exception, match='Sequence should be of same type'):
        load_with_rcl(wrapped(tmp_path, name))


def test_wrapped_basic_params_load_with_integer_gates(tmp_path):
    params = load_with_rcl(wrapped(tmp_path, 'basic_params.yaml'))
    assert params['mahalanobis_d_sonar'].type_ == Parameter.Type.INTEGER
    assert params['mocap_override_channel'].type_ == Parameter.Type.INTEGER
    assert params['enable_mocap_switch'].value is True


def test_converted_config_has_exactly_the_legacy_values():
    legacy = legacy_values()
    converted = yaml.safe_load(MASTER.read_text())['/**']['ros__parameters']
    assert sorted(converted) == sorted(legacy)
    for key, want in legacy.items():
        got = converted[key]
        if isinstance(want, bool):
            assert got is want, key
        elif isinstance(want, list):
            assert len(got) == len(want), key
            assert all(float(g) == float(w) for g, w in zip(got, want)), key
            assert all(isinstance(g, float) for g in got), f'{key}: write every element as a float'
        elif key == 'mocap_override_channel':
            assert got == want and isinstance(got, int), key
        else:
            assert float(got) == float(want) and isinstance(got, float), key


def test_converted_config_loads_under_ros2_as_double_arrays():
    params = load_with_rcl(MASTER)
    for key, value in yaml.safe_load(MASTER.read_text())['/**']['ros__parameters'].items():
        if isinstance(value, list):
            assert params[key].type_ == Parameter.Type.DOUBLE_ARRAY, key


def test_simulation_disables_rc_switch_explicitly():
    sim = yaml.safe_load(SIM.read_text())['/**']['ros__parameters']
    assert sim['enable_mocap_switch'] is False
    assert load_with_rcl(MASTER, SIM)['enable_mocap_switch'].value is False


def test_rcraw_has_the_legacy_layout():
    cls = get_message('rosflight_msgs/msg/RCRaw')
    assert cls.get_fields_and_field_types() == {'header': 'std_msgs/Header', 'values': 'uint16[8]'}
    assert list(cls().values) == [0] * 8
