"""The generated ROS 2 messages have the legacy fields (reef_msgs 7fb63ff).

DesiredState and DesiredVector were added in P06 (reef_control).

LEGACY is transcribed from the ROS 1 .msg files at 7fb63ff, in their field
order. The only allowed differences are RENAMED (ROS 2 field names must be
lower case) and `Header` becoming `std_msgs/Header` (ROS 2 has no `seq`).
"""
import pytest
from rosidl_runtime_py.utilities import get_message

LEGACY = {
    'DeltaToVel': [
        ('header', 'Header'), ('S_upper_bound', 'float64[6]'), ('S_lower_bound', 'float64[6]'),
        ('scaled_std_xyz', 'float64[3]'), ('vel', 'geometry_msgs/TwistWithCovarianceStamped')],
    'DesiredVector': [('x', 'float64'), ('y', 'float64'), ('z', 'float64'), ('yaw', 'float64')],
    'DesiredState': [
        ('header', 'Header'), ('node_id', 'uint32'), ('pose', 'DesiredVector'),
        ('velocity', 'DesiredVector'), ('acceleration', 'DesiredVector'),
        ('attitude', 'DesiredVector'), ('attitude_valid', 'bool'), ('position_valid', 'bool'),
        ('velocity_valid', 'bool'), ('acceleration_valid', 'bool'), ('altitude_only', 'bool')],
    'XYEstimate': [('x_dot', 'float64'), ('y_dot', 'float64')],
    'ZEstimate': [('z', 'float64'), ('z_dot', 'float64')],
    'XYZEstimate': [
        ('header', 'Header'), ('node_id', 'uint32'),
        ('xy_plus', 'XYEstimate'), ('z_plus', 'ZEstimate')],
    'XYDebugEstimate': [
        ('x_dot', 'float64'), ('y_dot', 'float64'), ('pitch_bias', 'float64'),
        ('roll_bias', 'float64'), ('xa_bias', 'float64'), ('ya_bias', 'float64'),
        ('sigma_plus', 'float64[6]'), ('sigma_minus', 'float64[6]')],
    'ZDebugEstimate': [
        ('z', 'float64'), ('z_dot', 'float64'), ('bias', 'float64'), ('u', 'float64'),
        ('P', 'float64[9]'), ('truth', 'float64[2]'), ('z_error', 'float64'),
        ('z_dot_error', 'float64'), ('sigma_plus', 'float64[3]'), ('sigma_minus', 'float64[3]')],
    'XYZDebugEstimate': [
        ('header', 'Header'), ('node_id', 'uint32'),
        ('xy_minus', 'XYDebugEstimate'), ('xy_plus', 'XYDebugEstimate'),
        ('z_minus', 'ZDebugEstimate'), ('z_plus', 'ZDebugEstimate')],
}
RENAMED = {('DeltaToVel', 'S_upper_bound'): 's_upper_bound',
           ('DeltaToVel', 'S_lower_bound'): 's_lower_bound',
           ('ZDebugEstimate', 'P'): 'p'}
NOT_PORTED = {'SyncEstimateError', 'SyncVerifyEstimates'}  # DesiredState/Vector: P06


def ros2_type(msg, legacy_type):
    """Expected rosidl type string for a legacy field type."""
    if legacy_type == 'Header':
        return 'std_msgs/Header'
    base, _, size = legacy_type.partition('[')
    base = {'float64': 'double'}.get(base, base)
    if base in LEGACY:
        base = f'reef_msgs/{base}'
    return f'{base}[{size}' if size else base


@pytest.mark.parametrize('name', sorted(LEGACY))
def test_fields_match_legacy(name):
    cls = get_message(f'reef_msgs/msg/{name}')
    expected = [(RENAMED.get((name, f), f), ros2_type(name, t)) for f, t in LEGACY[name]]
    assert list(cls.get_fields_and_field_types().items()) == expected


def test_exactly_three_renames():
    assert len(RENAMED) == 3


@pytest.mark.parametrize('name', sorted(NOT_PORTED))
def test_unused_messages_not_ported(name):
    with pytest.raises((AttributeError, ModuleNotFoundError, ValueError)):
        get_message(f'reef_msgs/msg/{name}')


def test_unset_fields_are_zero():
    msg = get_message('reef_msgs/msg/XYZDebugEstimate')()
    assert msg.node_id == 0
    assert list(msg.z_plus.truth) == [0.0, 0.0]
    assert msg.z_plus.z_error == 0.0 and msg.z_plus.z_dot_error == 0.0
    assert msg.header.frame_id == ''
    assert len(msg.z_plus.p) == 9
