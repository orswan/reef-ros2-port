#!/usr/bin/env python3
"""Print this environment's private colcon tree: build/colcon_env/<env>/.

The project tree is bind-mounted into both containers, whose ROS packages
differ. A CMake cache configured in one container references libraries that
may not exist in the other (P03: 'No rule to make target libfastcdr.so...'),
so scripts that build C++ packages use a tree per environment. <env> is a
short hash of the installed package list (dpkg), so it changes when the
image changes. The tree holds build/, install/, and log/.
"""
import hashlib
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def environment_id():
    out = subprocess.run(['dpkg-query', '-W', '-f=${Package}\\t${Version}\\n'],
                         capture_output=True, text=True, check=True).stdout
    return hashlib.sha256(''.join(sorted(out.splitlines(True))).encode()).hexdigest()[:12]


def tree():
    return ROOT / 'build' / 'colcon_env' / environment_id()


if __name__ == '__main__':
    print(tree())
    sys.exit(0)
