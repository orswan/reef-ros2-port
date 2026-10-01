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
  baseline [--floor]            Numerical parity. P02: build the pinned original
                                (master e4179f48, sim 95987b51) in the reference
                                harness, 15 fixtures x 2 parameter sets,
                                independent re-derivation, invariants, analytic
                                and characterization checks, golden; --floor adds
                                -O0/FMA builds. P05: the ported estimator (built
                                in the per-environment tree) vs the original on
                                50 event streams, every Z and XY field, wrapper
                                equivalence, negatives, D1/C1 accounting (about
                                15 min)
  interfaces                    P03 messages, helpers, parameters: vendored
                                rosflight_msgs pin (+ tampered-copy negative),
                                legacy reef_msgs helper vectors reproduce,
                                colcon build + test of src/ with minimum test
                                counts, baseline golden unchanged (about 5 min)
  estimator                     Physical plausibility (P04/P05): colcon build +
                                tests; X3 simulation + REEF scored against truth
                                (idealized inputs: truth attitude, idealized
                                range, simulated velocity observations, IMU
                                vibration assumption); parity with the original
                                on that recorded stream; two offline replays
                                bit-identical; ROS replay isolated from foreign
                                clock/sensor streams (about 12 min)
  faults                        F1-F12 (ACCEPTANCE.md section 5) on fixtures, the
                                node, replays, and an own simulation run; P07b
                                closed-loop scenarios: estimate dropout (short,
                                long: crash expected), estimator reset,
                                controller restart, stale setpoint, range and
                                velocity loss, pause/resume, stand-in exit
                                (crash expected), position mode with K9/K10
                                (about 30 min)
  control                       P06 controller fidelity: colcon build + tests;
                                the pinned original reef_control in a reference
                                harness vs the port (core and node) on 20
                                streams, independent model, K1-K12
                                characterizations, parameter cases, negative
                                control, plus a stream from an own X3 + REEF run;
                                dry-run sink in the tests. P07 closed loop: REEF
                                estimator + controller fly the X3 through a
                                stand-in low-level loop (development tool) on
                                idealized inputs; nominal run scored against the
                                P07 criteria and a causality run with a range
                                bias (about 15 min)
  vision                        P08 RGB-D (in progress): colcon build + tests;
                                rgbd_to_velocity: the pinned original in a
                                reference harness vs the port (core and node),
                                independent model, legacy quirks Q1-Q10,
                                negative control. Camera, odometry, REEF on
                                vision, degraded and closed-loop cases: N/A
                                until implemented (about 3 min)
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
  release) not_implemented release P09 ;;
  env|clock|sim-data|baseline|interfaces|estimator|faults|control|vision) ;;
  *) echo "unknown target '$target'"; usage; exit 2 ;;
esac
if [[ "$target" == env || "$target" == interfaces || "$target" == estimator || "$target" == faults || "$target" == control || "$target" == vision ]] && (( gui || regress || floor )); then
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
    # P05: the port against the original (numerical parity).
    tree="$(python3 "$S/colcon_tree.py")"
    configs+=("$B/fixtures_vertical.lock.json" "$B/fixtures_horizontal.lock.json" "$B/tools/check_port.py")
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "port build (per-environment tree)" 0 "$logdir/port_build.log" \
      bash -c 'cd "$1" && colcon --log-base "$2/log" build --base-paths src --symlink-install --build-base "$2/build" --install-base "$2/install" --packages-up-to reef_estimator --event-handlers console_cohesion-' \
      _ "$REEF_ROOT" "$tree"
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "port vs original: 50 streams, all fields, wrapper, C1" 0 "$logdir/port_parity.log" \
      bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/reef_estimator/lib/reef_estimator/reef_estimator_event_replay"' \
      _ "$tree" "$B/tools/check_port.py"
    grep -E '^(PASS|FAIL) \[(negative|accounting)\]|^(PASS|FAIL): ' "$logdir/port_parity.log" | sed 's/^/     /' || true
    artifacts+=("$REEF_ROOT/build/baseline/port_parity/fixtures/results.json")
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

  estimator)
    tree="$(python3 "$S/colcon_tree.py")"
    configs+=("$REEF_ROOT/src/reef_estimator/config/estimator_master.yaml"
              "$REEF_ROOT/src/reef_estimator/config/simulation.yaml"
              "$REEF_ROOT/src/reef_sim/config/x3_scenario.yaml" "$REEF_ROOT/src/reef_sim/config/x3_reef_overlay.yaml")
    "$REEF_ROOT/baseline/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    python3 "$S/setup_assets.py" --verify >"$logdir/assets_precheck.log" 2>&1 \
      || blocked "X3 assets missing or modified; run scripts/setup_assets.py"
    run_step "colcon build + test (unit, node, launch tests)" 0 "$logdir/colcon.log" \
      python3 "$S/check_colcon.py"
    run="$logdir/x3_reef"
    run_step "simulation: X3 + REEF vs truth (idealized inputs)" 0 "$logdir/sim.log" \
      env REEF_X3_OUT="$run" "$S/run_x3_scenario.sh" --estimator
    grep -E '^(PASS|FAIL) (takeoff|no landing|outputs|covariance|altitude|vertical velocity|horizontal|bias|estimate age|callback)' "$logdir/sim.log" | sed 's/^/     /' || true
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "offline replay of that run (core, full covariances)" 0 "$logdir/offline.log" \
      bash -c 'source "$1/install/setup.bash" && ros2 run reef_sim x3_reef_offline "$2" && ros2 run reef_sim analyze_reef_vertical "$2" --offline "$2/reef_offline"' \
      _ "$tree" "$run"
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "recorded simulation stream: port = original" 0 "$logdir/stream_parity.log" \
      bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/reef_estimator/lib/reef_estimator/reef_estimator_event_replay" --stream "$3/reef_offline/params.params" "$3/reef_offline/inputs.events" x3_run' \
      _ "$tree" "$REEF_ROOT/baseline/tools/check_port.py" "$run"
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "two offline replays bit-identical" 0 "$logdir/determinism.log" \
      bash -c 'source "$1/install/setup.bash" && ros2 run reef_sim x3_reef_offline "$2" --out "$2/reef_offline_repeat" >/dev/null && cmp "$2/reef_offline/estimates.csv" "$2/reef_offline_repeat/estimates.csv" && echo "identical: $(wc -c < "$2/reef_offline/estimates.csv") bytes"' \
      _ "$tree" "$run"
    run_step "ROS replay (no foreign clock or sensor streams)" 0 "$logdir/replay.log" \
      "$S/replay_reef_estimator.sh" "$run"
    span="$(python3 - "$run/scenario_result.json" 2>/dev/null <<'PY' || echo '?'
import json, sys
r = json.load(open(sys.argv[1]))
print(f"{r['phases'][0]['t_start']:.2f} -> {r['phases'][-1]['t_end']:.2f} s (scenario phases)")
PY
)"
    sim_notes+=("$span")
    artifacts+=("$run/analysis_reef" "$run/reef_offline/analysis_reef")
    ;;

  faults)
    tree="$(python3 "$S/colcon_tree.py")"
    configs+=("$REEF_ROOT/scripts/check_faults.py" "$REEF_ROOT/baseline/fixtures_horizontal.lock.json")
    "$REEF_ROOT/baseline/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    python3 "$S/setup_assets.py" --verify >"$logdir/assets_precheck.log" 2>&1 \
      || blocked "X3 assets missing or modified; run scripts/setup_assets.py"
    run="$logdir/x3_reef"
    run_step "simulation run for the fault cases (--estimator)" 0 "$logdir/sim.log" \
      env REEF_X3_OUT="$run" "$S/run_x3_scenario.sh" --estimator
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "fault cases F1-F12" 0 "$logdir/faults.log" \
      bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --tree "$1" --run "$3"' \
      _ "$tree" "$S/check_faults.py" "$run"
    grep -E '^(PASS|FAIL) \[(F[0-9]+|determinism)\]|^(PASS|FAIL): ' "$logdir/faults.log" | grep -v 'parity and wrapper' | sed 's/^/     /' || true
    # P07b: closed-loop fault and position-mode scenarios (stand-in low-level
    # loop, idealized inputs; crashes are the documented legacy result where
    # the legacy system has no protection).
    for sc in dropout_short dropout_long estimator_reset controller_restart setpoint_stale range_loss \
              velocity_loss pause_resume standin_exit position_square position_face_target; do
      run_step "closed loop P07b: $sc" 0 "$logdir/cl_$sc.log" \
        env REEF_X3_OUT="$logdir/cl_$sc" REEF_X3_CL_SCENARIO="$sc" "$S/run_x3_scenario.sh" --closed-loop
      grep -E '^FAIL |CHARACTERIZATION' "$logdir/cl_$sc.log" | cut -c1-160 | sed 's/^/     /' || true
      artifacts+=("$logdir/cl_$sc/analysis_closed_loop")
    done
    configs+=("$REEF_ROOT/src/reef_sim/config/x3_closed_loop.yaml")
    sim_notes+=("fixture time, the simulation run in ${run#"$REEF_ROOT"/}, and 11 closed-loop scenario runs (sim time)")
    ;;

  control)
    tree="$(python3 "$S/colcon_tree.py")"
    C="$REEF_ROOT/baseline/control"
    configs+=("$C/provenance.json" "$C/fixtures.lock.json" "$C/fixtures.py" "$C/control_model.py"
              "$C/check_control.py" "$REEF_ROOT/src/reef_control/config/reef_control_quad.yaml")
    "$REEF_ROOT/baseline/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    python3 "$S/setup_assets.py" --verify >"$logdir/assets_precheck.log" 2>&1 \
      || blocked "X3 assets missing or modified; run scripts/setup_assets.py"
    run_step "colcon build + test (package set, minimum test counts)" 0 "$logdir/colcon.log" \
      python3 "$S/check_colcon.py"
    grep -E '^reef_control +files=' "$logdir/colcon.log" | sed 's/^/     /' || true
    run="$logdir/x3_reef"
    # shellcheck disable=SC2016  # expanded by the inner shell
    # Only the estimate stream is needed here; the estimator's plausibility
    # (including its timing limits) is judged by `reef_check.sh estimator`.
    run_step "X3 + REEF run and offline replay (estimate stream)" 0 "$logdir/sim.log" \
      bash -c 'REEF_X3_OUT="$2" "$3/run_x3_scenario.sh" --estimator --no-analysis && source "$1/install/setup.bash" && ros2 run reef_sim x3_reef_offline "$2"' \
      _ "$tree" "$run" "$S"
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "controller: original vs port, model, characterizations" 0 "$logdir/control.log" \
      bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/reef_control/lib/reef_control/reef_control_event_replay" --stream "$3/reef_offline/estimates.csv"' \
      _ "$tree" "$C/check_control.py" "$run"
    grep -E '^(PASS|FAIL) \[(K|params|negative|cfg)\]|^(PASS|FAIL): ' "$logdir/control.log" | sed 's/^/     /' || true
    # P07: REEF estimator + controller in the loop, stand-in low-level loop (development tool).
    cl="$logdir/closed_loop_nominal"
    run_step "closed loop: nominal run, P07 criteria" 0 "$logdir/closed_loop.log" \
      env REEF_X3_OUT="$cl" "$S/run_x3_scenario.sh" --closed-loop
    grep -E '^(PASS|FAIL) ' "$logdir/closed_loop.log" | sed 's/^/     /' || true
    run_step "closed loop: causality run (+0.30 m range bias)" 0 "$logdir/closed_loop_bias.log" \
      env REEF_X3_OUT="$logdir/closed_loop_bias" REEF_X3_RANGE_BIAS=0.30 REEF_X3_NOMINAL="$cl" \
      "$S/run_x3_scenario.sh" --closed-loop
    grep -E '^(PASS|FAIL) causality' "$logdir/closed_loop_bias.log" | sed 's/^/     /' || true
    configs+=("$REEF_ROOT/src/reef_sim/config/x3_closed_loop.yaml" "$REEF_ROOT/src/reef_control/config/reef_control_x3_sim.yaml"
              "$REEF_ROOT/src/reef_fc_standin/config/x3_standin.yaml" "$REEF_ROOT/src/reef_sim/worlds/x3_closed_loop.sdf")
    sim_notes+=("fixture time; estimate stream from ${run#"$REEF_ROOT"/}; closed-loop runs in sim time (stand-in low-level loop, idealized inputs)")
    artifacts+=("$REEF_ROOT/build/baseline/control/check/results.json" "$run/reef_offline/estimates.csv"
                "$cl/analysis_closed_loop" "$logdir/closed_loop_bias/analysis_closed_loop")
    ;;

  vision)
    tree="$(python3 "$S/colcon_tree.py")"
    V="$REEF_ROOT/baseline/rgbd"
    configs+=("$V/provenance.json" "$V/fixtures.lock.json" "$V/fixtures.py" "$V/rgbd_model.py" "$V/check_rgbd.py"
              "$REEF_ROOT/src/rgbd_to_velocity/config/kiwi_camera.yaml")
    "$REEF_ROOT/baseline/fetch_sources.sh" >"$logdir/sources_precheck.log" 2>&1 \
      || blocked "pinned upstream sources unavailable (see ${logdir#"$REEF_ROOT"/}/sources_precheck.log)"
    run_step "colcon build + test (package set, minimum test counts)" 0 "$logdir/colcon.log" \
      python3 "$S/check_colcon.py"
    grep -E '^rgbd_to_velocity +files=' "$logdir/colcon.log" | sed 's/^/     /' || true
    # shellcheck disable=SC2016  # expanded by the inner shell
    run_step "rgbd_to_velocity: original vs port, model, quirks Q1-Q10" 0 "$logdir/rgbd.log" \
      bash -c 'source "$1/install/setup.bash" && exec python3 "$2" --port "$1/install/rgbd_to_velocity/lib/rgbd_to_velocity/rgbd_to_velocity_event_replay"' \
      _ "$tree" "$V/check_rgbd.py"
    grep -E '^(PASS|FAIL) \[(Q|negative)\]|^(PASS|FAIL): ' "$logdir/rgbd.log" | cut -c1-150 | sed 's/^/     /' || true
    for part in "camera interface" "replacement odometry vs truth (open loop)" "REEF on vision (open loop)" \
                "weak texture / depth loss / delayed and missing frames" "closed loop on vision" "performance"; do
      results+=("N/A|$part|NOT IMPLEMENTED yet (P08 in progress)")
    done
    sim_notes+=("fixture time only so far")
    artifacts+=("$REEF_ROOT/build/baseline/rgbd/check/results.json")
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
