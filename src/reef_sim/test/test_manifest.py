"""manifest.yaml names what actually commands the motors: the stock
truth-fed controller in an open-loop run, REEF through the stand-in in a
closed-loop one (H11 finding, 2026-10-06)."""
import importlib.util
import re
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]


def manifest_module():
    spec = importlib.util.spec_from_file_location('x3_manifest', ROOT / 'scripts' / 'x3_manifest.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def world_plugin_names(name):
    # Comments are stripped before parsing: the generated vision worlds carry
    # "--" inside one, which Gazebo's parser tolerates and ElementTree rejects.
    text = re.sub(r'<!--.*?-->', '', (PKG / 'worlds' / name).read_text(), flags=re.S)
    model = ET.fromstring(text).find('world/include')
    return [p.get('name') for p in model.findall('plugin')]


def test_open_loop_label_names_the_stock_controller():
    label = manifest_module().controller_label({'closed_loop': 'false', 'vision': 'false'})
    assert 'MulticopterVelocityControl' in label and 'TRUTH' in label
    assert 'REEF controller' not in label
    # The run it describes really does carry the stock controller.
    assert 'gz::sim::systems::MulticopterVelocityControl' in world_plugin_names('x3_flight.sdf')


def test_closed_loop_label_names_reef_and_the_standin():
    label = manifest_module().controller_label({'closed_loop': 'true', 'vision': 'false'})
    assert 'REEF controller' in label
    assert 'STAND-IN' in label and 'not ROSflight' in label
    assert 'fed by SIMULATION TRUTH' not in label
    # The worlds a closed-loop run uses have no stock controller to credit.
    for world in ('x3_closed_loop.sdf', 'x3_closed_loop_vision.sdf'):
        assert 'gz::sim::systems::MulticopterVelocityControl' not in world_plugin_names(world)


def test_missing_flag_falls_back_to_the_stock_controller():
    assert manifest_module().controller_label({}) == \
        manifest_module().controller_label({'closed_loop': 'false'})
