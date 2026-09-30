#!/usr/bin/env bash
# REEF project demonstrations. Run `scripts/reef_demo.sh help`. Container
# terminal only. Every mode is SIMULATION ONLY: no mode talks to hardware, and
# none exists that could.
#
# Exit status: 0 the demo ran to completion, 1 it ran and failed,
# 2 NOT IMPLEMENTED / BLOCKED / invalid invocation, 130/143 interrupted.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

S="$REEF_ROOT/scripts"

usage() {
  cat <<'EOF'
Usage: scripts/reef_demo.sh help
       scripts/reef_demo.sh MODE [options]

Modes (simulation only):
  stock [--gui]       X3 quadrotor flown by Gazebo's stock velocity controller,
                      which uses SIMULATION TRUTH (not REEF). Records and
                      analyzes the flight (scripts/run_x3_scenario.sh).
                      --gui shows it at http://127.0.0.1:8081/vnc.html
  replay RUN_DIR [--rate R]
                      Play recordings/<run>/bag in its own ROS domain (printed),
                      without /x3/cmd_vel, driven by the bag's recorded /clock.
                      Attach consumers in that domain with use_sim_time:=true.
                      REEF_REPLAY_DOMAIN=<n> selects the domain.
  estimator [--gui]   X3 flown by the stock truth-fed controller with the ported
                      REEF estimator (vertical + horizontal) running beside it on
                      IDEALIZED inputs (truth attitude, idealized range, simulated
                      velocity observations from truth, IMU vibration assumption).
                      REEF is not in the control loop.
                      Records, scores against truth, plots (analysis_reef/).
  estimator --offline RUN_DIR
                      Deterministic replay of a recording: adapter + ported core,
                      no ROS graph and no /clock (RUN_DIR/reef_offline/).
  estimator --replay RUN_DIR [--rate R]
                      Replay on a ROS graph in its own domain; the bag is the only
                      /clock source and no foreign sensor streams exist (checked)
                      (RUN_DIR/reef_replay_*/).
  closed-loop         NOT IMPLEMENTED (milestone P07)
  vision              NOT IMPLEMENTED (milestone P08)

Exit: 0 completed, 1 failed, 2 NOT IMPLEMENTED / BLOCKED / invalid,
130/143 interrupted.
EOF
}

not_implemented() { echo "NOT IMPLEMENTED: demo '$1' is not available yet (milestone $2). Nothing was run."; exit 2; }

mode="${1:-}"
shift || true
if [[ -z "$mode" ]]; then usage; echo; echo "missing mode (see above)"; exit 2; fi
case "$mode" in
  help|-h|--help) usage; exit 0 ;;
  closed-loop) not_implemented closed-loop P07 ;;
  vision) not_implemented vision P08 ;;

  stock)
    args=()
    for a in "$@"; do
      case "$a" in
        --gui) args+=(--gui) ;;
        *) echo "invalid option '$a' for 'stock'"; usage; exit 2 ;;
      esac
    done
    python3 "$S/setup_assets.py" --verify >/dev/null 2>&1 \
      || { echo "BLOCKED: X3 assets missing or modified; run scripts/setup_assets.py"; exit 2; }
    if [[ " ${args[*]:-} " == *" --gui "* ]] && ! "$S/check_display.sh" >/dev/null 2>&1; then
      echo "BLOCKED: browser desktop not ready (scripts/check_display.sh)"; exit 2
    fi
    echo "SIMULATION: X3 flown by the stock truth-fed controller (not REEF control)."
    rc=0
    "$S/run_x3_scenario.sh" "${args[@]}" || rc=$?
    case "$rc" in
      0|130|143) exit "$rc" ;;
      *) exit 1 ;;   # run_x3_scenario.sh codes 1/2/3/124: the demo ran and failed
    esac
    ;;

  estimator)
    rc=0
    case "${1:-}" in
      --offline)
        run="${2:-}"
        [[ -n "$run" && -f "$run/bag/metadata.yaml" && $# -eq 2 ]] || { echo "usage: estimator --offline RUN_DIR"; exit 2; }
        tree="$(python3 "$S/colcon_tree.py")"
        [[ -f "$tree/install/setup.bash" ]] || { echo "BLOCKED: build first (scripts/reef_check.sh estimator or run_x3_scenario.sh --estimator)"; exit 2; }
        set +u
        # shellcheck disable=SC1091
        source "$tree/install/setup.bash"
        set -u
        echo "SIMULATION REPLAY (offline, deterministic) of $run: IDEALIZED INPUTS."
        ros2 run reef_sim x3_reef_offline "$run" || rc=1
        (( rc == 0 )) && { ros2 run reef_sim analyze_reef_vertical "$run" --offline "$run/reef_offline" || rc=1; }
        ;;
      --replay)
        shift
        "$S/replay_reef_estimator.sh" "$@" || rc=$?
        ;;
      ""|--gui)
        args=()
        for a in "$@"; do
          case "$a" in
            --gui) args+=(--gui) ;;
            *) echo "invalid option '$a' for 'estimator'"; usage; exit 2 ;;
          esac
        done
        python3 "$S/setup_assets.py" --verify >/dev/null 2>&1 \
          || { echo "BLOCKED: X3 assets missing or modified; run scripts/setup_assets.py"; exit 2; }
        if [[ " ${args[*]:-} " == *" --gui "* ]] && ! "$S/check_display.sh" >/dev/null 2>&1; then
          echo "BLOCKED: browser desktop not ready (scripts/check_display.sh)"; exit 2
        fi
        echo "SIMULATION: X3 flown by the stock truth-fed controller; the REEF estimator runs beside it on"
        echo "IDEALIZED INPUTS (truth attitude, idealized range, simulated velocity observations from truth,"
        echo "IMU vibration assumption). REEF is not in the control loop."
        "$S/run_x3_scenario.sh" --estimator "${args[@]}" || rc=$?
        ;;
      *) echo "invalid option '$1' for 'estimator'"; usage; exit 2 ;;
    esac
    case "$rc" in
      0|2|130|143) exit "$rc" ;;
      *) exit 1 ;;
    esac
    ;;

  replay)
    run="${1:-}"; shift || true
    rate=1.0
    while (( $# )); do
      case "$1" in
        --rate) rate="${2:-}"; shift 2 || { echo "--rate needs a value"; exit 2; } ;;
        *) echo "invalid option '$1' for 'replay'"; usage; exit 2 ;;
      esac
    done
    [[ -n "$run" ]] || { echo "replay needs a run directory (recordings/<run>)"; exit 2; }
    [[ "$rate" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "invalid --rate '$rate'"; exit 2; }
    bag="$run/bag"
    [[ -f "$bag/metadata.yaml" ]] || { echo "BLOCKED: no finalized bag at $bag"; exit 2; }
    export ROS_DOMAIN_ID="${REEF_REPLAY_DOMAIN:-$(( RANDOM % 101 + 1 ))}"
    echo "SIMULATION REPLAY of $bag"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID (discovery: $ROS_AUTOMATIC_DISCOVERY_RANGE), rate $rate, /x3/cmd_vel excluded"
    echo "  The bag's recorded /clock drives time. In another container terminal:"
    echo "    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "    ros2 topic echo /x3/range --once"
    echo "  Do not run a live simulation in this domain (two /clock sources)."
    rc=0
    ros2 bag play "$bag" --exclude-topics /x3/cmd_vel --rate "$rate" || rc=$?
    case "$rc" in
      0|130|143) exit "$rc" ;;
      *) exit 1 ;;
    esac
    ;;

  *) echo "unknown mode '$mode'"; usage; exit 2 ;;
esac
