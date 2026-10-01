"""x3_closed_loop.sdf is x3_flight.sdf without the stock controller (P07), and
the stand-in's geometry and motor constants match the model and world."""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]


def plugins(path):
    model = ET.parse(path).getroot().find('world/include')
    return [(p.get('name'), {c.tag: (c.text or '').strip() for c in p}) for p in model.findall('plugin')]


def test_stock_controller_removed_and_motor_models_identical():
    flight = plugins(PKG / 'worlds' / 'x3_flight.sdf')
    closed = plugins(PKG / 'worlds' / 'x3_closed_loop.sdf')
    names = [n for n, _ in closed]
    assert 'gz::sim::systems::MulticopterVelocityControl' not in names
    motor = 'gz::sim::systems::MulticopterMotorModel'
    assert [p for n, p in flight if n == motor] == [p for n, p in closed if n == motor]
    assert len([n for n in names if n == motor]) == 4
    odom = [p for n, p in closed if n == 'gz::sim::systems::OdometryPublisher']
    assert {o['odom_topic'] for o in odom} == {'/reef/x3/truth/odom', '/reef/x3/fc/odom'}


def test_standin_matches_the_model():
    cfg = yaml.safe_load((ROOT / 'src' / 'reef_fc_standin' / 'config' / 'x3_standin.yaml').read_text())
    p = cfg['/**']['ros__parameters']
    motors = [pp for n, pp in plugins(PKG / 'worlds' / 'x3_closed_loop.sdf')
              if n == 'gz::sim::systems::MulticopterMotorModel']
    model = ET.parse(PKG / 'models' / 'reef_x3' / 'model.sdf').getroot().find('model')
    for i, m in enumerate(motors):
        assert int(m['actuator_number']) == i
        assert float(m['motorConstant']) == p['motor_constant']
        assert float(m['momentConstant']) == p['moment_constant']
        assert float(m['maxRotVelocity']) == p['max_rot_velocity']
        assert p['rotor_dir'][i] == (1 if m['turningDirection'] == 'ccw' else -1)
        link = next(lk for lk in model.findall('link') if lk.get('name') == m['linkName'])
        x, y = (float(v) for v in re.split(r'\s+', link.find('pose').text.strip())[:2])
        assert (p['rotor_x'][i], p['rotor_y'][i]) == (x, -y)   # FLU -> FRD
    inertia = model.find("link[@name='X3/base_link']/inertial/inertia")
    assert (p['ixx'], p['iyy'], p['izz']) == tuple(float(inertia.find(k).text) for k in ('ixx', 'iyy', 'izz'))
