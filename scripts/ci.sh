#!/usr/bin/env bash
# Local continuous checks (P09, ACCEPTANCE `release`, CI). Container terminal.
#
#   scripts/ci.sh            # fast profile (default): deterministic checks and one short scenario
#   scripts/ci.sh --full     # also fresh-build compiler warnings and the ASan/UBSan gtests
#
# Fast profile (about 30 min on an idle machine; estimator parity alone about 19 min):
#   1. code quality, fast gates: shellcheck, pyflakes, suppressions
#   2. reef_check.sh interfaces: vendored pin, helper vectors, colcon build +
#      every unit/interface test (minimum counts), golden unchanged
#   3. estimator parity: the pinned original (reference harness) vs the port, bit-exact
#   4. controller parity: the pinned reef_control vs the port, bit-exact, K1-K12
#   5. rgbd_to_velocity parity: the pinned original vs the port, bit-exact, Q1-Q10
#   6. one short headless scenario: the stock X3 flight, recorded and analyzed
# Expensive suites (rendering, closed loop, faults, the release gate) stay
# explicit: scripts/reef_check.sh release [--profile vision], faults, vision.
#
# Output: log/ci/ci_<time>/ with summary.txt and junit.xml (one test case
# per step; the failure text is the log tail). Exit: 0 every step passed,
# 1 a step failed, 2 a prerequisite is missing (nothing judged), 130/143
# interrupted.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

S="$REEF_ROOT/scripts"
full=0
for a in "$@"; do
  case "$a" in
    --full) full=1 ;;
    *) echo "usage: scripts/ci.sh [--full]"; exit 2 ;;
  esac
done
cd "$REEF_ROOT" || exit 2
out="$REEF_ROOT/log/ci/ci_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
echo "ci: logs in ${out#"$REEF_ROOT"/}"
python3 "$S/setup_assets.py" --verify >"$out/assets_precheck.log" 2>&1 \
  || { echo "BLOCKED: X3 assets missing or modified; run scripts/setup_assets.py"; exit 2; }
"$REEF_ROOT/baseline/fetch_sources.sh" >"$out/sources_precheck.log" 2>&1 \
  || { echo "BLOCKED: pinned upstream sources unavailable; run baseline/fetch_sources.sh"; exit 2; }

steps=()   # "name|rc|seconds|log"
overall=0
child=""
# shellcheck disable=SC2317  # reached only via traps
on_signal() { [[ -n "$child" ]] && kill "-$1" "$child" 2>/dev/null && wait "$child" 2>/dev/null; echo "INTERRUPTED (SIG$1)"; exit "$2"; }
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM
step() {  # name log cmd...
  local name="$1" log="$2" rc=0 t0=$SECONDS; shift 2
  echo "---- $name"
  env --default-signal=INT "$@" </dev/null > >(tee "$log" | grep -E '^(PASS|FAIL|BLOCKED|=+ reef_check)' | cut -c1-140) 2>&1 &
  child=$!
  wait "$child" || rc=$?
  child=""
  sleep 0.2
  (( rc == 130 || rc == 143 )) && { echo "INTERRUPTED during '$name'"; exit "$rc"; }
  (( rc == 0 )) || overall=1
  steps+=("$name|$rc|$(( SECONDS - t0 ))|$log")
  echo "     -> $( ((rc == 0)) && echo PASS || echo "FAIL (exit $rc)" ), $(( SECONDS - t0 )) s"
}

tree="$(python3 "$S/colcon_tree.py")"
step "code quality (fast gates)" "$out/quality.log" "$S/check_code_quality.sh" --skip-build
step "reef_check.sh interfaces" "$out/interfaces.log" "$S/reef_check.sh" interfaces
# shellcheck disable=SC2016  # expanded by the inner shell
step "estimator parity (reference harness vs port)" "$out/estimator_parity.log" \
  bash -c '"$3/baseline/build_reference.sh" master sim && source "$1/install/setup.bash" && exec python3 "$3/baseline/tools/check_port.py" --port "$1/install/reef_estimator/lib/reef_estimator/reef_estimator_event_replay"' \
  _ "$tree" "" "$REEF_ROOT"
# shellcheck disable=SC2016  # expanded by the inner shell
step "controller parity (reference harness vs port)" "$out/control_parity.log" \
  bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/reef_control/lib/reef_control/reef_control_event_replay"' \
  _ "$tree" "$REEF_ROOT/baseline/control/check_control.py"
# shellcheck disable=SC2016  # expanded by the inner shell
step "rgbd_to_velocity parity (reference harness vs port)" "$out/rgbd_parity.log" \
  bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/rgbd_to_velocity/lib/rgbd_to_velocity/rgbd_to_velocity_event_replay"' \
  _ "$tree" "$REEF_ROOT/baseline/rgbd/check_rgbd.py"
step "short headless scenario (stock X3 flight, recorded, analyzed)" "$out/scenario.log" \
  env REEF_X3_OUT="$out/x3_scenario" "$S/run_x3_scenario.sh"
(( full )) && step "code quality (fresh-build warnings, ASan/UBSan gtests)" "$out/quality_full.log" "$S/check_code_quality.sh"

python3 - "$out" "$full" "$overall" "${steps[@]}" <<'EOF'
import subprocess, sys
from pathlib import Path
from xml.sax.saxutils import escape
out, full, overall, rows = Path(sys.argv[1]), sys.argv[2] == '1', int(sys.argv[3]), sys.argv[4:]
commit = subprocess.run(['git', 'rev-parse', '--short=12', 'HEAD'], capture_output=True, text=True).stdout.strip()
dirty = len(subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True).stdout.splitlines())
cases, lines = [], [f"ci ({'full' if full else 'fast'}) at {commit}, {dirty} uncommitted/untracked path(s): "
                    f"{'PASS' if overall == 0 else 'FAIL'}"]
fails = 0
for r in rows:
    name, rc, sec, log = r.split('|', 3)
    ok = rc == '0'
    fails += not ok
    lines.append(f"  {'PASS' if ok else 'FAIL'} {name} (exit {rc}, {sec} s) {Path(log).name}")
    body = ''
    if not ok:
        tail = ''.join(Path(log).read_text(errors='replace').splitlines(True)[-40:])
        body = f'<failure message="exit {rc}">{escape(tail)}</failure>'
    cases.append(f'  <testcase classname="reef_ci" name="{escape(name)}" time="{sec}">{body}</testcase>')
(out / 'summary.txt').write_text('\n'.join(lines) + '\n')
(out / 'junit.xml').write_text('<?xml version="1.0" encoding="UTF-8"?>\n'
                               f'<testsuite name="reef_ci" tests="{len(rows)}" failures="{fails}">\n'
                               + '\n'.join(cases) + '\n</testsuite>\n')
print('\n'.join(lines))
EOF
echo "reports: ${out#"$REEF_ROOT"/}/summary.txt, junit.xml"
exit "$overall"
