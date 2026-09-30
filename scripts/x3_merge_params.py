#!/usr/bin/env python3
"""Merge a parameter overlay into an X3 scenario parameter file, in place.

    x3_merge_params.py RUN_PARAMS.yaml OVERLAY.yaml

For each node in the overlay, its ros__parameters are added to (or replace
those of) the same node in RUN_PARAMS. Used by run_x3_scenario.sh --estimator
so the run directory holds the exact merged parameters. Exit 0 merged, 2 error.
"""
import sys
from pathlib import Path

import yaml


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    run, overlay = Path(sys.argv[1]), Path(sys.argv[2])
    base = yaml.safe_load(run.read_text())
    extra = yaml.safe_load(overlay.read_text())
    for node, section in extra.items():
        base.setdefault(node, {}).setdefault('ros__parameters', {}).update(section['ros__parameters'])
    run.write_text(f'# merged: scenario parameters + {overlay.name} (run_x3_scenario.sh --estimator)\n'
                   + yaml.safe_dump(base, sort_keys=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
