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

The project tree is bind-mounted into both containers, whose ROS packages
differ. A CMake cache configured in one container references libraries that
may not exist in the other, so this check never uses the shared build/ and
install/. It builds in build/colcon_env/<env>/ (scripts/colcon_tree.py), where
<env> is a hash of the installed package list (dpkg), so each environment has
its own tree.

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
sys.path.insert(0, str(ROOT / 'scripts'))
import colcon_tree  # noqa: E402

EXPECTED = {'reef_msgs', 'reef_estimator', 'reef_sim', 'rosflight_msgs'}
# package -> (minimum test cases, required result-file kinds)
MINIMUM = {
    'reef_msgs': (49, {'gtest', 'xunit'}),
    'reef_estimator': (60, {'gtest', 'xunit'}),
    'reef_sim': (12, {'pytest'}),
}
BASE = ['--base-paths', 'src']


ENV_ID = colcon_tree.environment_id()
TREE = colcon_tree.tree()
BUILD, INSTALL = TREE / 'build', TREE / 'install'
COLCON = ['colcon', '--log-base', str(TREE / 'log')]
BASES = ['--build-base', str(BUILD), '--install-base', str(INSTALL)]


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
    print(f'environment {ENV_ID}: build tree {TREE.relative_to(ROOT)}')
    if run(COLCON + ['build', '--symlink-install', '--executor', 'sequential',
                     '--event-handlers', 'console_cohesion-'] + BASES + BASE, env=env).returncode != 0:
        print(f'FAIL colcon build (see {(TREE / "log").relative_to(ROOT)}/latest_build/<pkg>/stdout_stderr.log)')
        return 1

    for p in pkgs:   # stale results must not count
        shutil.rmtree(BUILD / p / 'test_results', ignore_errors=True)
        (BUILD / p / 'pytest.xml').unlink(missing_ok=True)
    test_rc = run(COLCON + ['test', '--executor', 'sequential', '--return-code-on-test-failure',
                            '--event-handlers', 'console_cohesion-'] + BASES + BASE, env=env).returncode
    if test_rc != 0:
        failures.append(f'colcon test exited {test_rc}')
    result_rc = run(COLCON + ['test-result', '--verbose', '--test-result-base', str(BUILD)]).returncode
    if result_rc != 0:
        failures.append(f'colcon test-result exited {result_rc}')

    grand = 0
    for p in sorted(pkgs):
        files = sorted((BUILD / p / 'test_results').rglob('*.xml'))
        files = [f for f in files if f.name.endswith(('.gtest.xml', '.xunit.xml'))]
        kinds = {f.name.rsplit('.', 2)[-2] for f in files}
        if (BUILD / p / 'pytest.xml').is_file():
            files.append(BUILD / p / 'pytest.xml')
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
