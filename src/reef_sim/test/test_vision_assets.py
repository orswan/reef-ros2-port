"""P08 vision assets: generated files up to date, and the simulated camera's
extrinsics equal the converter's simulation parameters (ACCEPTANCE.md
vision, camera interface)."""
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]


def test_generated_assets_are_up_to_date():
    r = subprocess.run([sys.executable, str(ROOT / 'scripts' / 'make_vision_assets.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


def rot(v, w):
    """The converter's quaternion formula (VISION.md section 2): C = I - 2 w [v]x + 2 [v]x^2."""
    s = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)
    return np.eye(3) - 2 * w * s + 2 * s @ s


def test_camera_extrinsics_match_the_converter():
    model = ET.parse(PKG / 'models' / 'reef_x3_rgbd' / 'model.sdf').getroot().find('model')
    sensor = model.find("link[@name='X3/base_link']/sensor[@name='rgbd']")
    pose = [float(x) for x in re.split(r'\s+', sensor.find('pose').text.strip())]
    assert pose[3:] == [0.0, 0.0, 0.0]                      # Gazebo camera axes = body FLU (x forward)
    p = yaml.safe_load((ROOT / 'src' / 'rgbd_to_velocity' / 'config' / 'x3_sim_camera.yaml').read_text())
    p = p['/**']['ros__parameters']
    assert p['body_to_camera_trans'] == [pose[0], -pose[1], -pose[2]]   # FLU -> FRD
    q = p['body_to_camera_quat']
    C = rot(np.array(q[:3]), q[3])
    # Body FRD -> DEMO camera (x left, y up, z forward) for a forward camera with x along body x.
    expected = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)
    assert np.allclose(C, expected, atol=1e-15)
