#!/usr/bin/env bash
# Build the reference harness for one or both pinned estimator variants.
#   baseline/build_reference.sh [master|sim ...]      (default: both)
# Environment: REF_CXXFLAGS (default "-O2"), REF_TAG (binary name suffix),
# REF_MUTATE (test only: a sed expression applied to the extracted
# src/z_estimator.cpp to build a deliberately wrong variant for the
# negative check; never set for reference builds).
# Sources are extracted from the pinned Git objects into build/baseline/ in
# the original directory layout, checked against baseline/provenance.json,
# and compiled UNMODIFIED against the ROS stand-in headers in
# baseline/harness/shim. Adaptations: see baseline/README.md.
# Exit: 0 built, 1 compile or checksum failure, 2 sources unavailable.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
B="$ROOT/baseline"
"$B/fetch_sources.sh" >/dev/null || exit 2
variants=("$@"); (( ${#variants[@]} )) || variants=(master sim)
flags="${REF_CXXFLAGS:--O2}"
tag="${REF_TAG:-}"

for v in "${variants[@]}"; do
  out="$ROOT/build/baseline/$v"
  tree="$out/tree"
  rm -rf "$tree"; mkdir -p "$tree"
  python3 - "$B/provenance.json" "$v" "$ROOT/reference" "$tree" <<'PY' || exit 1
import hashlib, json, subprocess, sys
from pathlib import Path
prov, v, ref, tree = json.load(open(sys.argv[1])), sys.argv[2], Path(sys.argv[3]), Path(sys.argv[4])
if v not in prov['variants']:
    sys.exit(f'unknown variant {v}')
jobs = [('reef_estimator', prov['variants'][v]['commit'], prov['variants'][v]['files']),
        ('reef_msgs', prov['reef_msgs']['commit'], prov['reef_msgs']['files'])]
for repo, commit, files in jobs:
    for path, digest in files.items():
        data = subprocess.run(['git', '-C', str(ref / repo), 'show', f'{commit}:{path}'],
                              capture_output=True, check=True).stdout
        if hashlib.sha256(data).hexdigest() != digest:
            sys.exit(f'CHECKSUM MISMATCH {repo}@{commit}:{path}')
        dest = tree / repo / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
PY
  E="$tree/reef_estimator" M="$tree/reef_msgs"
  if [[ -n "${REF_MUTATE:-}" ]]; then
    [[ -n "$tag" ]] || { echo "REF_MUTATE requires REF_TAG"; exit 1; }
    before="$(sha256sum "$E/src/z_estimator.cpp")"
    sed -i "$REF_MUTATE" "$E/src/z_estimator.cpp"
    [[ "$(sha256sum "$E/src/z_estimator.cpp")" != "$before" ]] || { echo "REF_MUTATE changed nothing"; exit 1; }
    echo "MUTATED build ($tag): sed '$REF_MUTATE' on src/z_estimator.cpp"
  fi
  bin="$out/reef_ref${tag:+_$tag}"
  # A1: access specifiers relaxed for the REEF classes only (after all library
  #     headers; see harness/access_prelude.h), identically in every
  #     translation unit, so the harness can read filter internals.
  # shellcheck disable=SC2086  # REF_CXXFLAGS is a flag list
  g++ -std=c++17 $flags -include "$B/harness/access_prelude.h" -w \
    -I "$B/harness/shim" -I "$M/include" -I "$E/include" \
    "$E/src/estimator.cpp" "$E/src/xy_estimator.cpp" "$E/src/z_estimator.cpp" \
    "$E/src/xyz_estimator.cpp" "$E/src/sensor_manager.cpp" \
    "$M/src/dynamics.cpp" "$M/src/matrix_operation.cpp" \
    "$B/harness/reef_ref_main.cpp" -o "$bin" || { echo "FAIL compile $v"; exit 1; }
  echo "OK   built ${bin#"$ROOT"/} ($v $(python3 -c "import json;print(json.load(open('$B/provenance.json'))['variants']['$v']['commit'][:8])"), flags: $flags)"
done
