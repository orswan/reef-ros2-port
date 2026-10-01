#!/usr/bin/env bash
# Build the rgbd_to_velocity reference harness from the pinned sources (P08).
#   baseline/rgbd/build_rgbd_reference.sh
# Environment: REF_CXXFLAGS (default "-O2"), REF_TAG (binary name suffix),
# REF_MUTATE (test only: a sed expression applied to the extracted
# src/rgbd_to_velocity.cpp for the negative check; never set for reference builds).
# Sources are extracted from the pinned Git objects into build/baseline/rgbd/
# in the original catkin layout (the converter includes reef_msgs through
# ../../reef_msgs/...), checked against baseline/rgbd/provenance.json, and
# compiled UNMODIFIED against baseline/rgbd/shim then baseline/harness/shim.
# Adaptations V1-V2: baseline/README.md. /usr/include/eigen3 is on the include
# path as catkin find_package(Eigen) put it (the original includes <Eigen/Geometry>).
# Exit: 0 built, 1 compile or checksum failure, 2 sources unavailable.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
B="$ROOT/baseline"
"$B/fetch_sources.sh" >/dev/null || exit 2
flags="${REF_CXXFLAGS:--O2}"
tag="${REF_TAG:-}"
out="$ROOT/build/baseline/rgbd"
tree="$out/tree"
rm -rf "$tree"; mkdir -p "$tree"
python3 - "$B/rgbd/provenance.json" "$ROOT/reference" "$tree" <<'PY' || exit 1
import hashlib, json, subprocess, sys
from pathlib import Path
prov, ref, tree = json.load(open(sys.argv[1])), Path(sys.argv[2]), Path(sys.argv[3])
for repo in ('rgbd_to_velocity', 'reef_msgs'):
    commit = prov[repo]['commit']
    for path, digest in prov[repo]['files'].items():
        data = subprocess.run(['git', '-C', str(ref / repo), 'show', f'{commit}:{path}'],
                              capture_output=True, check=True).stdout
        if hashlib.sha256(data).hexdigest() != digest:
            sys.exit(f'CHECKSUM MISMATCH {repo}@{commit}:{path}')
        dest = tree / repo / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
PY
R="$tree/rgbd_to_velocity"
if [[ -n "${REF_MUTATE:-}" ]]; then
  [[ -n "$tag" ]] || { echo "REF_MUTATE requires REF_TAG"; exit 1; }
  before="$(sha256sum "$R/src/rgbd_to_velocity.cpp")"
  sed -i "$REF_MUTATE" "$R/src/rgbd_to_velocity.cpp"
  [[ "$(sha256sum "$R/src/rgbd_to_velocity.cpp")" != "$before" ]] || { echo "REF_MUTATE changed nothing"; exit 1; }
  echo "MUTATED build ($tag): sed '$REF_MUTATE' on src/rgbd_to_velocity.cpp"
fi
bin="$out/rgbd_ref${tag:+_$tag}"
# shellcheck disable=SC2086  # REF_CXXFLAGS is a flag list
g++ -std=c++17 $flags -fno-lifetime-dse -w \
  -I "$B/rgbd/shim" -I "$B/harness/shim" -I "$R/include" -I /usr/include/eigen3 \
  "$R/src/rgbd_to_velocity.cpp" "$B/rgbd/rgbd_ref_main.cpp" -o "$bin" || { echo "FAIL compile rgbd reference"; exit 1; }
echo "OK   built ${bin#"$ROOT"/} (rgbd_to_velocity b7637198, flags: $flags)"
