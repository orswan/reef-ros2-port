#!/usr/bin/env bash
# Build the controller reference harness from the pinned reef_control sources.
#   baseline/control/build_control_reference.sh
# Environment: REF_CXXFLAGS (default "-O2"), REF_TAG (binary name suffix),
# REF_MUTATE (test only: a sed expression applied to the extracted
# src/simple_pid.cpp to build a deliberately wrong variant for the negative
# check; never set for reference builds).
# Sources are extracted from the pinned Git objects into build/baseline/control/
# in the original layout, checked against baseline/control/provenance.json,
# and compiled UNMODIFIED against the stand-in headers in
# baseline/control/shim (then baseline/harness/shim). Adaptations C1-C3:
# see baseline/README.md.
# Exit: 0 built, 1 compile or checksum failure, 2 sources unavailable.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
B="$ROOT/baseline"
"$B/fetch_sources.sh" >/dev/null || exit 2
flags="${REF_CXXFLAGS:--O2}"
tag="${REF_TAG:-}"
out="$ROOT/build/baseline/control"
tree="$out/tree"
rm -rf "$tree"; mkdir -p "$tree"
python3 - "$B/control/provenance.json" "$ROOT/reference" "$tree" <<'PY' || exit 1
import hashlib, json, subprocess, sys
from pathlib import Path
prov, ref, tree = json.load(open(sys.argv[1])), Path(sys.argv[2]), Path(sys.argv[3])
for repo in ('reef_control', 'reef_msgs'):
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
C="$tree/reef_control" M="$tree/reef_msgs"
if [[ -n "${REF_MUTATE:-}" ]]; then
  [[ -n "$tag" ]] || { echo "REF_MUTATE requires REF_TAG"; exit 1; }
  before="$(sha256sum "$C/src/simple_pid.cpp")"
  sed -i "$REF_MUTATE" "$C/src/simple_pid.cpp"
  [[ "$(sha256sum "$C/src/simple_pid.cpp")" != "$before" ]] || { echo "REF_MUTATE changed nothing"; exit 1; }
  echo "MUTATED build ($tag): sed '$REF_MUTATE' on src/simple_pid.cpp"
fi
bin="$out/control_ref${tag:+_$tag}"
# C1: access keywords relaxed after all library headers (control/access_prelude.h).
# -fno-lifetime-dse keeps C3 (zeroed storage before construction) effective.
# shellcheck disable=SC2086  # REF_CXXFLAGS is a flag list
g++ -std=c++17 $flags -fno-lifetime-dse -include "$B/control/access_prelude.h" -w \
  -I "$B/control/shim" -I "$B/harness/shim" -I "$M/include" -I "$C/include" \
  "$C/src/controller.cpp" "$C/src/PID.cpp" "$C/src/simple_pid.cpp" \
  "$B/control/control_ref_main.cpp" -o "$bin" || { echo "FAIL compile control reference"; exit 1; }
echo "OK   built ${bin#"$ROOT"/} (reef_control 12237b76, flags: $flags)"
