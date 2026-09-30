#!/usr/bin/env bash
# Record the legacy reef_msgs helper vectors used by the P03 port tests.
#   baseline/helper_vectors.sh                 regenerate and compare with the committed copy
#   baseline/helper_vectors.sh --update "why"  regenerate and overwrite the committed copy
# The legacy sources are extracted and checksummed by build_reference.sh
# (master tree) and compiled UNMODIFIED against baseline/harness/shim.
# -ffp-contract=off keeps the vectors independent of -march (no FMA).
# Exit: 0 identical/updated, 1 differs or build failure, 2 invalid use or sources unavailable.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
B="$ROOT/baseline"
COMMITTED="$ROOT/src/reef_msgs/test/data/legacy_helper_vectors.txt"
update_reason=""
case "${1:-}" in
  "") ;;
  --update) [[ -n "${2:-}" ]] || { echo "usage: $0 [--update \"<reason>\"]"; exit 2; }; update_reason="$2" ;;
  *) echo "usage: $0 [--update \"<reason>\"]"; exit 2 ;;
esac

rc=0; "$B/build_reference.sh" master >/dev/null || rc=$?
(( rc == 0 )) || { echo "FAIL legacy source extraction/build (exit $rc)"; exit "$rc"; }
M="$ROOT/build/baseline/master/tree/reef_msgs"
out="$ROOT/build/baseline/helper_vectors"
mkdir -p "$out"
g++ -std=c++17 -O2 -ffp-contract=off -w -I "$B/harness/shim" -I "$M/include" \
  "$M/src/dynamics.cpp" "$M/src/matrix_operation.cpp" "$B/harness/helper_vectors.cpp" \
  -o "$out/helper_vectors" || { echo "FAIL compile helper_vectors"; exit 1; }
"$out/helper_vectors" "$out/legacy_helper_vectors.txt"
counts="$(cut -d' ' -f1 "$out/legacy_helper_vectors.txt" | grep -v '^#' | sort | uniq -c | tr -s ' ' | tr '\n' ';')"

if [[ -n "$update_reason" ]]; then
  mkdir -p "$(dirname "$COMMITTED")"
  cp "$out/legacy_helper_vectors.txt" "$COMMITTED"
  echo "UPDATED ${COMMITTED#"$ROOT"/} ($counts) reason: $update_reason"
  exit 0
fi
if cmp -s "$out/legacy_helper_vectors.txt" "$COMMITTED"; then
  echo "PASS legacy helper vectors reproduce the committed copy ($counts)"
else
  echo "FAIL regenerated legacy vectors differ from ${COMMITTED#"$ROOT"/}"
  exit 1
fi
