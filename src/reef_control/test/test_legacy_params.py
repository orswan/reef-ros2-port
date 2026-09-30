"""config/reef_control_quad.yaml carries the values the original read from
config/legacy/quad_pid.yaml (reef_control 12237b76), with the documented
differences: face_target and fly_fixed_wing were ignored inside the
namespace (K12, effective false); integer switches were rejected by roscpp's
boolean getParam (cfg default true); gaussian_offset is not a parameter."""
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]


def test_quad_values_match_the_legacy_file():
    legacy = yaml.safe_load((PKG / 'config' / 'legacy' / 'quad_pid.yaml').read_text())['reef_control_pid']
    ros2 = yaml.safe_load((PKG / 'config' / 'reef_control_quad.yaml').read_text())['/**']['ros__parameters']
    expected = dict(legacy)
    expected.update(face_target=False, fly_fixed_wing=False, xIntegrator=True, uIntegrator=True)
    del expected['gaussian_offset']
    assert ros2 == expected
