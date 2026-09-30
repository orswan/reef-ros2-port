"""Run the reference harness over the fixtures and load the results."""
import csv
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'build' / 'baseline'
VARIANTS = ('master', 'sim')
KINDS = ('common', 'shipped')   # common: master's parameter values for both variants


def parse_params(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        if not line or line.startswith('#'):
            continue
        name, kind, *vals = line.split()
        if kind == 'bool':
            out[name] = vals[0] == 'true'
        elif kind == 'double':
            out[name] = float(vals[0])
        elif kind == 'string':
            out[name] = vals[0]
        else:
            out[name] = [float(v) for v in vals]
    return out


def parse_events(path):
    ev = []
    for line in Path(path).read_text().splitlines():
        if not line or line.startswith('#'):
            continue
        typ, t, *vals = line.split()
        ev.append([typ, int(t)] + [float(v) for v in vals])
    return ev


def read_rows(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == 'type' else float(v)) for k, v in r.items()})
    return rows


def fixtures_dir():
    return BUILD / 'fixtures'


def scenario_names():
    return sorted(p.stem for p in fixtures_dir().glob('*.events'))


def param_file(kind, scenario, variant):
    sets = json.loads((fixtures_dir() / 'manifest.json').read_text())['param_set_for_scenario']
    return fixtures_dir() / 'params' / f'{kind}_{sets.get(scenario, "default")}_{variant}.params'


def run(variant, kind, scenario, binary=None, out_dir=None):
    """Run one fixture; return the output CSV path."""
    binary = binary or BUILD / variant / 'reef_ref'
    out_dir = out_dir or BUILD / 'out' / variant / kind
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f'{scenario}.csv'
    subprocess.run([str(binary), str(param_file(kind, scenario, variant)),
                    str(fixtures_dir() / f'{scenario}.events'), str(out)], check=True)
    return out
