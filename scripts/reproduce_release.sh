#!/usr/bin/env bash
# Fresh-clone reproduction of the simulation release (P09, ACCEPTANCE
# `release`: fresh clone, pinned resources, offline, representative scores).
#
#   scripts/reproduce_release.sh [--profile core|vision] [--dest DIR]
#
# 1. Clones the committed HEAD into DIR/reef_ros2 (default
#    build/release_repro/<time>/, ignored by Git) with --no-local: no build
#    trees, no ignored files (assets/, reference/ clones, recordings), no
#    caches. Uncommitted changes are refused (they would not be reproduced).
# 2. Checks the clone is pristine and that nothing of this checkout leaks in
#    (no build/install/log/assets, reference/ holds only COLCON_IGNORE).
# 3. Setup, ONLINE, by the documented steps only (README "Quick start"):
#    scripts/setup_assets.py, then baseline/fetch_sources.sh.
# 4. OFFLINE (REEF_OFFLINE=1, scripts/env.sh): first proves the guard blocks
#    Git, curl and Python downloads (negative controls), then runs
#    scripts/reef_check.sh release --profile PROFILE inside the clone. It
#    includes the P07 closed-loop nominal run, whose metrics are reported
#    beside this checkout's latest release run.
# Writes DIR/reproduction_summary.{txt,json}.
# Exit: 0 reproduced (release gate PASS offline); 1 the gate failed in the
# clone; 2 refused or setup/guard failed (nothing judged); 130/143
# interrupted. Container terminal. shellcheck: from PATH or REEF_SHELLCHECK.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

profile=core dest=""
while (( $# )); do
  case "$1" in
    --profile) profile="${2:-}"; shift ;;
    --dest) dest="${2:-}"; shift ;;
    *) echo "usage: scripts/reproduce_release.sh [--profile core|vision] [--dest DIR]"; exit 2 ;;
  esac
  shift
done
[[ "$profile" == core || "$profile" == vision ]] || { echo "invalid profile '$profile'"; exit 2; }
cd "$REEF_ROOT" || exit 2
if [[ -n "$(git status --porcelain)" ]]; then
  git status --short | head -20
  echo "REFUSED: uncommitted changes would not be in the clone; commit them first"
  exit 2
fi
commit="$(git rev-parse HEAD)"
dest="${dest:-$REEF_ROOT/build/release_repro/$(date +%Y%m%d_%H%M%S)}"
[[ -e "$dest" ]] && { echo "REFUSED: $dest exists"; exit 2; }
mkdir -p "$dest"
clone="$dest/reef_ros2"
log="$dest/reproduce.log"
exec > >(tee "$log") 2>&1
echo "== reproduce release $commit (profile $profile) in $clone"
child=""
# shellcheck disable=SC2317  # reached only via traps
on_signal() { [[ -n "$child" ]] && kill "-$1" "$child" 2>/dev/null && wait "$child" 2>/dev/null; echo "INTERRUPTED (SIG$1)"; exit "$2"; }
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM
run() {  # rc of "$@" with signal forwarding; the clone's scripts re-exec cleanly themselves
  env --default-signal=INT REEF_CLEAN_ENV= "$@" </dev/null &
  child=$!
  local rc=0
  wait "$child" || rc=$?
  child=""
  (( rc == 130 || rc == 143 )) && { echo "INTERRUPTED"; exit "$rc"; }
  return "$rc"
}
checks=()   # "PASS|FAIL name: detail"
note() { checks+=("$1 $2"); echo "$1 $2"; }

# 1-2. fresh clone, pristine
if ! git clone -q --no-local --no-hardlinks "$REEF_ROOT" "$clone" || ! git -C "$clone" checkout -q --detach "$commit"; then
  echo "BLOCKED: clone failed"; exit 2
fi
leftovers="$(cd "$clone" && ls -d build install log assets recordings 2>/dev/null; find reference -mindepth 1 -maxdepth 1 ! -name COLCON_IGNORE 2>/dev/null)"
if [[ -n "$leftovers" || "$(git -C "$clone" rev-parse HEAD)" != "$commit" ]]; then
  echo "BLOCKED: the clone is not pristine: $leftovers"; exit 2
fi
note PASS "fresh clone: $commit, no build/install/log/assets/recordings, reference/ holds only COLCON_IGNORE"

# 3. setup, online, documented steps only
echo "== setup (online): scripts/setup_assets.py; baseline/fetch_sources.sh"
if run "$clone/scripts/setup_assets.py" && run "$clone/baseline/fetch_sources.sh"; then
  note PASS "setup: pinned X3 model (SHA-256) and upstream sources (commits) fetched by the documented steps"
else
  echo "BLOCKED: setup failed (network, or a pin mismatch)"; exit 2
fi
fuel_files_after_setup="$(find "$clone/assets" -type f | wc -l)"

# 4. offline
export REEF_OFFLINE=1
echo "== offline guard (REEF_OFFLINE=1): negative controls"
guard_ok=1
# shellcheck disable=SC2016  # expanded by the inner shell
run bash -c 'source "$1/scripts/env.sh" && reef_setup_env && ! timeout 30 git ls-remote https://github.com/uf-reef-avl/reef_msgs >/dev/null 2>&1 && ! curl -sS -m 10 -o /dev/null https://fuel.gazebosim.org 2>/dev/null && ! python3 -c "import urllib.request as u; u.urlopen(\"https://fuel.gazebosim.org\", timeout=10)" 2>/dev/null' _ "$clone" || guard_ok=0
(( guard_ok )) || { echo "BLOCKED: the offline guard does not block downloads; refusing to call the run offline"; exit 2; }
note PASS "offline guard: Git, curl and Python downloads all fail"
echo "== release gate in the clone, offline: scripts/reef_check.sh release --profile $profile"
gate=0
run "$clone/scripts/reef_check.sh" release --profile "$profile" || gate=$?
summary="$(find "$clone/log/checks" -maxdepth 1 -name 'reef_check_release_*' -printf '%T@ %p\n' 2>/dev/null \
  | sort -n | tail -1 | cut -d' ' -f2-)/release_summary.json"
if (( gate == 0 )); then note PASS "release gate ($profile) offline in the clone: exit 0"; else note FAIL "release gate ($profile) offline in the clone: exit $gate"; fi
fuel_now="$(find "$clone/assets" -type f | wc -l)"
if [[ "$fuel_now" == "$fuel_files_after_setup" ]]; then
  note PASS "no download during the offline phase: assets unchanged ($fuel_now files); every scenario manifest reports its Fuel fetches (must be 0)"
else
  note FAIL "assets changed during the offline phase ($fuel_files_after_setup -> $fuel_now files)"
fi

python3 - "$dest" "$clone" "$REEF_ROOT" "$commit" "$profile" "$gate" "$summary" "${checks[@]}" <<'EOF'
import json, sys
from pathlib import Path
dest, clone, here, commit, profile, gate, summary = sys.argv[1:8]
checks = sys.argv[8:]
def latest(root, pattern):
    c = sorted(Path(root).glob(pattern), key=lambda p: p.stat().st_mtime)
    return c[-1] if c else None
def cl_metrics(root):
    r = latest(root, 'log/checks/reef_check_control_*/closed_loop_nominal/analysis_closed_loop/results.json')
    if not r:
        return None
    d = json.loads(r.read_text())
    m = d['metrics']
    judged = [c for c in d['checks'] if c['criterion'] != 'REPORTED']
    vel = max(max(v['rmse_x'], v['rmse_y']) for v in m['velocity'].values())
    return dict(run=str(r.parent.parent.relative_to(root)), judged=f"{sum(c['ok'] for c in judged)}/{len(judged)}",
                takeoff_s=m['takeoff']['reef_takeoff_after_arm'], max_tilt=m['stability']['max_tilt'],
                worst_velocity_rmse=vel, hold_rmse_max=max(v['rmse'] for v in m['hold'].values()),
                age_p99=m['latency']['age_p99'], yaw_rate=m['yaw_rate']['mean'])
mine, ref = cl_metrics(clone), cl_metrics(here)
lines = [f'reproduction of {commit} (profile {profile}): {"PASS" if gate == "0" else "FAIL"}', *checks,
         'representative scenario (P07 closed-loop nominal), REPORTED: clone vs this checkout\'s latest control run']
for k in (mine or {}):
    if k != 'run':
        lines.append(f'  {k:20} clone {mine[k]!s:>22}   reference {(ref or {}).get(k)!s:>22}')
lines.append(f"  runs: clone {mine and mine['run']}, reference {ref and ref['run']}")
s = Path(summary)
out = dict(commit=commit, profile=profile, gate_exit=int(gate), clone=clone, checks=checks,
           release_summary=json.loads(s.read_text()) if s.exists() else None, closed_loop_clone=mine,
           closed_loop_reference=ref)
Path(dest, 'reproduction_summary.json').write_text(json.dumps(out, indent=1, default=str))
Path(dest, 'reproduction_summary.txt').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
EOF
echo "reports: ${dest#"$REEF_ROOT"/}/reproduction_summary.txt, reproduction_summary.json, reproduce.log"
(( gate == 0 )) && exit 0
exit 1
