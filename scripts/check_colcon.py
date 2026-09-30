#!/usr/bin/env python3
"""Build and test the colcon workspace and judge the results (P03 `interfaces`).

1. `colcon list --base-paths src` must select exactly the expected packages
   (and no rosflight package other than rosflight_msgs).
2. `colcon build` of all selected packages.
3. Old result files are deleted, then `colcon test` runs; results are read
   from build/<pkg>/test_results (ament_cmake) and build/<pkg>/pytest.xml
   (ament_python). Every package with tests must meet its
   minimum number of executed test cases, with 0 failures, errors, and skips.
   An empty test run is a FAIL.

Expects a sourced ROS 2 environment (reef_check.sh provides it).
Exit: 0 PASS, 1 FAIL, 2 invalid environment.
"""
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {'reef_msgs', 'reef_estimator', 'reef_sim', 'rosflight_msgs'}
# package -> (minimum test cases, required result-file kinds)
MINIMUM = {
    'reef_msgs': (20, {'gtest', 'xunit'}),
    'reef_estimator': (20, {'gtest', 'xunit'}),
    'reef_sim': (7, {'pytest'}),
}
COLCON = ['colcon']
BASE = ['--base-paths', 'src']


def run(cmd, **kw):
    print('$ ' + ' '.join(cmd), flush=True)
    return subprocess.run(cmd, cwd=ROOT, **kw)


def list_packages():
    out = run(COLCON + ['list', '--names-only'] + BASE, capture_output=True, text=True, check=True)
    return set(out.stdout.split())


def count(xml_path):
    """(tests, failures, errors, skipped) summed over <testsuite> elements."""
    root = ET.parse(xml_path).getroot()
    suites = [root] if root.tag == 'testsuite' else root.findall('testsuite')
    keys = ('tests', 'failures', 'errors', 'skipped')
    return [sum(int(s.get(k, 0) or 0) for s in suites) for k in keys]


def main():
    if not shutil.which('colcon') or 'AMENT_PREFIX_PATH' not in os.environ:
        print('check_colcon: colcon or a sourced ROS 2 environment is missing')
        return 2
    failures = []

    pkgs = list_packages()
    print(f'selected packages: {sorted(pkgs)}')
    extra_rosflight = sorted(p for p in pkgs if p.startswith('rosflight') and p != 'rosflight_msgs')
    if pkgs != EXPECTED:
        failures.append(f'package set {sorted(pkgs)} != expected {sorted(EXPECTED)}')
    if extra_rosflight:
        failures.append(f'unexpected rosflight packages (out of scope): {extra_rosflight}')

    env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL=os.environ.get('CMAKE_BUILD_PARALLEL_LEVEL', '2'))
    if run(COLCON + ['build', '--symlink-install', '--executor', 'sequential',
                     '--event-handlers', 'console_cohesion-'] + BASE, env=env).returncode != 0:
        print('FAIL colcon build (see log/latest_build/<pkg>/stdout_stderr.log)')
        return 1

    for p in pkgs:   # stale results must not count
        shutil.rmtree(ROOT / 'build' / p / 'test_results', ignore_errors=True)
        (ROOT / 'build' / p / 'pytest.xml').unlink(missing_ok=True)
    test_rc = run(COLCON + ['test', '--executor', 'sequential', '--return-code-on-test-failure',
                            '--event-handlers', 'console_cohesion-'] + BASE, env=env).returncode
    if test_rc != 0:
        failures.append(f'colcon test exited {test_rc}')
    result_rc = run(COLCON + ['test-result', '--verbose', '--test-result-base', 'build']).returncode
    if result_rc != 0:
        failures.append(f'colcon test-result exited {result_rc}')

    grand = 0
    for p in sorted(pkgs):
        files = sorted((ROOT / 'build' / p / 'test_results').rglob('*.xml'))
        files = [f for f in files if f.name.endswith(('.gtest.xml', '.xunit.xml'))]
        kinds = {f.name.rsplit('.', 2)[-2] for f in files}
        if (ROOT / 'build' / p / 'pytest.xml').is_file():
            files.append(ROOT / 'build' / p / 'pytest.xml')
            kinds.add('pytest')
        t = [0, 0, 0, 0]
        for f in files:
            t = [a + b for a, b in zip(t, count(f))]
        grand += t[0]
        print(f'{p:16s} files={len(files):2d} kinds={sorted(kinds)} tests={t[0]} '
              f'failures={t[1]} errors={t[2]} skipped={t[3]}')
        if p in MINIMUM:
            need, need_kinds = MINIMUM[p]
            if t[0] < need:
                failures.append(f'{p}: {t[0]} test cases executed, need >= {need}')
            if not need_kinds <= kinds:
                failures.append(f'{p}: result kinds {sorted(kinds)} lack {sorted(need_kinds - kinds)}')
        if t[1] or t[2] or t[3]:
            failures.append(f'{p}: {t[1]} failures, {t[2]} errors, {t[3]} skipped')
    if grand == 0:
        failures.append('no test cases executed')

    for f in failures:
        print(f'FAIL {f}')
    print(f'check_colcon: {"PASS" if not failures else "FAIL"} ({grand} test cases)')
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
