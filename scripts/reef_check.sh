#!/usr/bin/env bash
# REEF project checks: one entry point wrapping the existing check scripts.
# Run `scripts/reef_check.sh help` for targets. Container terminal only.
#
# Exit status (wrapper boundary):
#   0  PASS: every executed assertion passed
#   1  FAIL: a check executed and failed (including timeouts and a crashed
#      owned simulation)
#   2  BLOCKED / NOT IMPLEMENTED / invalid invocation: nothing was judged
#   130/143  interrupted by SIGINT/SIGTERM (passed through; the wrapped
#      script has already cleaned up its own processes)
# The wrapped scripts keep their own codes (for example check_clock_demo.sh:
# 2 = owned demo failed, 124 = timeout). They are mapped here: prerequisites
# are checked first and reported as BLOCKED, so any other nonzero code from an
# executed check is a FAIL.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

S="$REEF_ROOT/scripts"
t_start=$SECONDS
results=()      # "VERDICT|name|detail"
artifacts=()
configs=()      # files whose hashes identify the configuration/inputs
sim_notes=()
overall=0       # 0 pass, 1 fail
child=""

usage() {
  cat <<'EOF'
Usage: scripts/reef_check.sh help
       scripts/reef_check.sh TARGET [options]

Targets (simulation only; nothing here talks to hardware):
  env                           ROS/Gazebo environment and browser desktop
                                (check_env.sh, check_display.sh)
  clock [--gui] [--regress]     Gazebo->ROS clock with a test-owned simulation:
                                headless check + no-simulator negative case;
                                --gui adds the GUI check on the desktop;
                                --regress adds regress_clock_check.sh (28 cases:
                                foreign simulation, invalid display, SIGINT/
                                SIGTERM/timeout cleanup, ...)
  sim-data [--gui] [--regress]  X3 scenario: flight, recording, analysis
                                (run_x3_scenario.sh); --regress adds
                                regress_x3_scenario.sh (13 cases)
  baseline [--floor]            P02 estimator baseline: build the pinned original
                                estimator (master e4179f48, sim 95987b51) in the
                                reference harness, run 15 fixtures x 2 parameter
                                sets, independent step-wise re-derivation,
                                covariance invariants, analytic/characterization
                                checks, golden comparison (about 8 min);
                                --floor adds -O0/FMA builds (floating-point floor)
  interfaces                    P03 messages, helpers, parameters: vendored
                                rosflight_msgs pin (+ tampered-copy negative),
                                legacy reef_msgs helper vectors reproduce,
                                colcon build + test of src/ with minimum test
                                counts, baseline golden unchanged (about 5 min)
  estimator                     NOT IMPLEMENTED (milestones P04-P05)
  faults                        NOT IMPLEMENTED (milestone P05)
  control                       NOT IMPLEMENTED (milestones P06-P07)
  vision                        NOT IMPLEMENTED (milestone P08)
  release                       NOT IMPLEMENTED (milestone P09)

Exit: 0 PASS, 1 FAIL (executed), 2 BLOCKED / NOT IMPLEMENTED / invalid,
130/143 interrupted. Logs: log/checks/reef_check_<target>_<time>/.
EOF
}

not_implemented() {  # target milestone
  echo "NOT IMPLEMENTED: '$1' has no check yet (milestone $2). Nothing was run."
  exit 2
}

blocked() { echo "BLOCKED: $*"; echo "Nothing was judged."; exit 2; }

# --- signal handling: forward to the running check, then report interruption
# shellcheck disable=SC2317  # reached only via traps
on_signal() {
  local sig="$1" code="$2"
  [[ -n "$child" ]] && kill "-$sig" "$child" 2>/dev/null && wait "$child" 2>/dev/null
  echo "INTERRUPTED (SIG$sig)"
  exit "$code"
}
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

run_step() {  # name expected_rc log cmd...: run a check, record PASS/FAIL against expected_rc
  local name="$1" expected="$2" log="$3"; shift 3
  local t0=$SECONDS rc=0
  echo "---- $name"
  # Background + wait so signals can be forwarded; stdin from /dev/null so no
  # step can be stopped by reading the terminal; default SIGINT disposition.
  env --default-signal=INT "$@" </dev/null > >(tee "$log") 2>&1 &
  child=$!
  wait "$child" || rc=$?
  child=""
  sleep 0.2   # let tee flush
  if (( rc == 130 || rc == 143 )); then echo "INTERRUPTED during '$name'"; exit "$rc"; fi
  local verdict=FAIL
  (( rc == expected )) && verdict=PASS
  [[ "$verdict" == PASS ]] || overall=1
  results+=("$verdict|$name|exit $rc (expected $expected), wall $(( SECONDS - t0 )) s")
  artifacts+=("$log")
}

sha() { sha256sum "$1" 2>/dev/null | cut -c1-12; }

report() {
  local target="$1" f r
  echo
  echo "================ reef_check $target: $( ((overall == 0)) && echo PASS || echo FAIL ) ================"
  local dirty
  dirty="$(git -C "$REEF_ROOT" status --porcelain | wc -l)"
  echo "source      : $(git -C "$REEF_ROOT" rev-parse --short=12 HEAD) ($(git -C "$REEF_ROOT" rev-parse --abbrev-ref HEAD)), $dirty uncommitted/untracked path(s)"
  if [[ -s /etc/reef-image-packages.txt ]]; then
    echo "environment : reef_ros2_dev image (package list sha256 $(sha /etc/reef-image-packages.txt)), gz sim $(gz sim --versions 2>/dev/null)"
  else
    echo "environment : not the dev image (no /etc/reef-image-packages.txt), gz sim $(gz sim --versions 2>/dev/null)"
  fi
  for f in "${configs[@]}"; do echo "config      : ${f#"$REEF_ROOT"/} sha256 $(sha "$f")"; done
  echo "assertions  :"
  for r in "${results[@]}"; do
    IFS='|' read -r v n d <<<"$r"
    printf '  %-4s %-58s %s\n' "$v" "$n" "$d"
  done
  for r in "${sim_notes[@]}"; do echo "sim time    : $r"; done
  echo "wall time   : $(( SECONDS - t_start )) s"
  for f in "${artifacts[@]}"; do echo "artifact    : ${f#"$REEF_ROOT"/}"; done
  exit "$overall"
}

target="${1:-}"
shift || true
if [[ -z "$target" ]]; then usage; echo; echo "missing target (see above)"; exit 2; fi
gui=0 regress=0 floor=0
for a in "$@"; do
  case "$a" in
    --gui) gui=1 ;;
    --regress) regress=1 ;;
    --floor) floor=1 ;;
    *) echo "invalid option '$a' for target '$target'"; usage; exit 2 ;;
  esac
done
case "$target" in
  help|-h|--help) usage; exit 0 ;;
  estimator) not_implemented estimator P04-P05 ;;
  faults) not_implemented faults P05 ;;
  control) not_implemented control P06-P07 ;;
  vision) not_implemented vision P08 ;;
  release) not_implemented release P09 ;;
  env|clock|sim-data|baseline|interfaces) ;;
  *) echo "unknown target '$target'"; usage; exit 2 ;;
esac
if [[ "$target" == env || "$target" == interfaces ]] && (( gui || regress || floor )); then
  echo "target '$target' takes no options"; exit 2
fi
if [[ "$target" == baseline ]] && (( gui || regress )); then echo "target 'baseline' accepts only --floor"; exit 2; fi
if [[ "$target" != baseline ]] && (( floor )); then echo "--floor applies only to 'baseline'"; exit 2; fi

logdir="$REEF_ROOT/log/checks/reef_check_${target}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$logdir"
echo "reef_check $target: logs in ${logdir#"$REEF_ROOT"/}"

case "$target" in
  env)
    configs+=("$REEF_ROOT/Dockerfile" "$REEF_ROOT/compose.yaml")
    run_step "environment (check_env.sh)" 0 "$logdir/env.log" "$S/check_env.sh"
    run_step "browser desktop (check_display.sh)" 0 "$logdir/display.log" "$S/check_display.sh"
    ;;

  clock)
    configs+=("$REEF_ROOT/sim/worlds/clock_demo.sdf" "$REEF_ROOT/sim/launch/clock_demo.launch.py"
              "$S/check_clock_demo.sh" "$S/clock_check.py")
    if (( gui )); then
      "$S/check_display.sh" >"$logdir/display_precheck.log" 2>&1 \
        || blocked "browser desktop not ready for --gui (see ${logdir#"$REEF_ROOT"/}/display_precheck.log)"
    fi
    run_step "headless: owned sim time advances in ROS 2" 0 "$logdir/headless.log" \
      env REEF_HEADLESS=1 "$S/check_clock_demo.sh"
    (( gui )) && run_step "GUI: owned sim time advances in ROS 2" 0 "$logdir/gui.log" "$S/check_clock_demo.sh"
    # Negative case: nothing publishes on a fresh topic; the observer must exit 1.
    run_step "negative: no simulator -> observer exits 1" 1 "$logdir/negative.log" \
      python3 "$S/clock_check.py" 1 3 --topic "/reef_check_$$/clock"
    for f in "$logdir/headless.log" "$logdir/gui.log"; do
      [[ -f "$f" ]] && sim_notes+=("$(basename "$f" .log): $(grep -m1 'clock sim time' "$f" | sed 's/.*: *//')")
    done
    (( regress )) && run_step "regress_clock_check.sh (28 cases)" 0 "$logdir/regress.log" "$S/regress_clock_check.sh"
    ;;

  baseline)
    B="$REEF_ROOT/baseline"
    configs+=("$B/provenance.json" "$B/fixtures.lock.json" "$B/golden/index.json" "$B/tools/fixtures.py"
              "$B/tools/independent.py" "$B/tools/check_baseline.py" "$B/harness/reef_ref_main.cpp")
    "$B/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    args=(); (( floor )) && args=(--floor)
    run_step "P02 baseline: reference harness, independent, analytic, golden" 0 "$logdir/baseline.log" \
      python3 "$B/tools/check_baseline.py" "${args[@]}"
    artifacts+=("$REEF_ROOT/build/baseline/report/summary.md" "$REEF_ROOT/build/baseline/report/results.json"
                "$REEF_ROOT/build/baseline/out")
    sim_notes+=("not applicable (fixture time only): $(grep -m1 -oE '[0-9]+ runs, [0-9]+ rows' "$logdir/baseline.log" || echo '?')")
    grep -E '^(PASS|FAIL): [0-9]+/[0-9]+ assertions' "$logdir/baseline.log" | sed 's/^/     /' || true
    ;;

  interfaces)
    V="$REEF_ROOT/src/third_party/rosflight_ros_pkgs"
    configs+=("$V/UPSTREAM.json" "$REEF_ROOT/src/reef_msgs/test/data/legacy_helper_vectors.txt"
              "$REEF_ROOT/src/reef_estimator/config/estimator_master.yaml"
              "$REEF_ROOT/src/reef_estimator/config/simulation.yaml")
    "$REEF_ROOT/baseline/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    run_step "rosflight_msgs is byte-identical to upstream v2.0.1" 0 "$logdir/vendor.log" \
      python3 "$S/check_vendor.py"
    cp -a "$V" "$logdir/tampered_vendor"
    printf ' ' >> "$logdir/tampered_vendor/rosflight_msgs/msg/RCRaw.msg"
    run_step "negative: a one-byte change to RCRaw.msg is detected" 1 "$logdir/vendor_negative.log" \
      python3 "$S/check_vendor.py" --dir "$logdir/tampered_vendor"
    run_step "legacy reef_msgs helper vectors reproduce" 0 "$logdir/helper_vectors.log" \
      "$REEF_ROOT/baseline/helper_vectors.sh"
    run_step "colcon build + test (package set, minimum test counts)" 0 "$logdir/colcon.log" \
      python3 "$S/check_colcon.py"
    run_step "baseline golden and fixture lock unchanged since P02" 0 "$logdir/golden.log" \
      git -C "$REEF_ROOT" diff --exit-code --stat 04c9b19 -- baseline/golden baseline/fixtures.lock.json
    grep -E '^(reef_|rosflight_)[a-z_]* +files=|^check_colcon:' "$logdir/colcon.log" | sed 's/^/     /' || true
    tree="$(grep -m1 -oE 'build tree [^ ]+' "$logdir/colcon.log" | cut -d' ' -f3)"
    [[ -n "$tree" ]] && artifacts+=("$REEF_ROOT/$tree/build/<pkg>/test_results" "$REEF_ROOT/$tree/log")
    sim_notes+=("not applicable (unit tests only)")
    ;;

  sim-data)
    configs+=("$REEF_ROOT/src/reef_sim/config/x3_scenario.yaml" "$REEF_ROOT/src/reef_sim/worlds/x3_flight.sdf"
              "$REEF_ROOT/src/reef_sim/models/reef_x3/model.sdf" "$REEF_ROOT/src/reef_sim/assets/x3_uav_v4.json")
    python3 "$S/setup_assets.py" --verify >"$logdir/assets_precheck.log" 2>&1 \
      || blocked "X3 assets missing or modified; run scripts/setup_assets.py (see ${logdir#"$REEF_ROOT"/}/assets_precheck.log)"
    if (( gui )); then
      "$S/check_display.sh" >"$logdir/display_precheck.log" 2>&1 \
        || blocked "browser desktop not ready for --gui (see ${logdir#"$REEF_ROOT"/}/display_precheck.log)"
    fi
    args=(); (( gui )) && args=(--gui)
    run_step "X3 scenario: fly, record, analyze" 0 "$logdir/scenario.log" "$S/run_x3_scenario.sh" "${args[@]}"
    run_dir="$(sed -n 's/^run directory: //p' "$logdir/scenario.log" | tail -1)"
    if [[ -n "$run_dir" ]]; then
      artifacts+=("$run_dir/manifest.yaml" "$run_dir/bag" "$run_dir/analysis")
      sim_notes+=("$(python3 - "$run_dir/scenario_result.json" 2>/dev/null <<'EOF' || echo "no readable scenario_result.json"
import json, sys
d = json.load(open(sys.argv[1]))
t = (d.get('final_truth') or {}).get('t', float('nan'))
print(f"scenario {d.get('scenario_duration_s', 0):.0f} s commanded, ended at sim t = {t:.2f} s ({d.get('status')})")
EOF
)")
    fi
    (( regress )) && run_step "regress_x3_scenario.sh (13 cases)" 0 "$logdir/regress.log" "$S/regress_x3_scenario.sh"
    ;;
esac
report "$target"
