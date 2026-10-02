#!/usr/bin/env bash
# Code-quality gates (P09, ACCEPTANCE `release`, code quality):
#   1. shellcheck -x on every shell script in Git (.shellcheckrc applies)
#   2. pyflakes on the project's Python (src/third_party/ excluded)
#   3. suppressions: every shellcheck/noqa/NOLINT/pragma directive is listed
#      and must carry a reason on the same line
#   4. compiler warnings: a FRESH build of every package (tree
#      <colcon_tree>-fresh, removed first) must report zero warnings in the
#      project's own sources (src/third_party/ excluded); packages build with
#      -Wall -Wextra -Wpedantic
#   5. sanitizers: scripts/check_sanitizers.sh (ASan + UBSan gtests)
#
#   scripts/check_code_quality.sh [--skip-build]   # --skip-build: gates 1-3 only (CI fast path)
#
# The shellcheck binary comes from PATH, or REEF_SHELLCHECK=<path> (the original container has
# none installed; the dev image does). Exit: 0 all gates pass, 1 a gate
# failed, 2 a tool is missing or the build could not run. Container terminal.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

skip_build=0
for a in "$@"; do
  case "$a" in
    --skip-build) skip_build=1 ;;
    *) echo "usage: scripts/check_code_quality.sh [--skip-build]"; exit 2 ;;
  esac
done
cd "$REEF_ROOT" || exit 2
rc=0
fail() { echo "FAIL $*"; rc=1; }
pass() { echo "PASS $*"; }

# 1. shellcheck
sc="${REEF_SHELLCHECK:-$(command -v shellcheck || true)}"
[[ -x "$sc" ]] || { echo "BLOCKED: shellcheck not found (install it, or set REEF_SHELLCHECK=<path>)"; exit 2; }
# tracked and untracked (not ignored) files, so new files are checked before they are committed
files() { git ls-files --cached --others --exclude-standard "$@"; }
mapfile -t shells < <({ files '*.sh'; files | while read -r f; do
  [[ "$f" == *.* || ! -f "$f" ]] && continue
  head -c 64 "$f" | grep -qE '^#!.*(bash|/sh)' && echo "$f"; done; } | grep -v '^src/third_party/' | sort -u)
if out="$("$sc" -x "${shells[@]}" 2>&1)"; then
  pass "shellcheck $("$sc" --version | sed -n 's/^version: //p'): ${#shells[@]} scripts, no findings"
else
  echo "$out" | head -40
  fail "shellcheck: findings in $(echo "$out" | grep -c '^In ') place(s)"
fi

# 2. pyflakes
python3 -m pyflakes --version >/dev/null 2>&1 || { echo "BLOCKED: pyflakes not installed (python3-pyflakes)"; exit 2; }
mapfile -t pys < <(files '*.py' | grep -v '^src/third_party/')
if out="$(python3 -m pyflakes "${pys[@]}" 2>&1)"; then
  pass "pyflakes $(python3 -m pyflakes --version | head -1 | cut -d' ' -f1): ${#pys[@]} files, no findings"
else
  echo "$out" | head -40
  fail "pyflakes: $(echo "$out" | wc -l) finding(s)"
fi

# 3. suppressions: listed, each with a reason on the same line
pattern='shellcheck (disable|source)=|# *noqa|NOLINT|pragma GCC diagnostic|type: *ignore|pylint: *disable'
mapfile -t supp < <(files | grep -v '^src/third_party/' | grep -v '^scripts/check_code_quality.sh$' \
  | xargs grep -n -E "$pattern" 2>/dev/null)
bare="$(printf '%s\n' "${supp[@]}" | grep -E "shellcheck (disable|source)=[A-Za-z0-9,]+ *$|# *noqa(: *[A-Z0-9, ]+)? *$|NOLINT(\([^)]*\))? *$" || true)"
echo "suppressions (${#supp[@]}):"
printf '%s\n' "${supp[@]}" | sed -E 's/^([^:]+:[0-9]+):\s*/  \1: /' | cut -c1-150
if [[ -n "$bare" ]]; then
  while IFS= read -r l; do echo "  NO REASON: $l"; done <<<"$bare"
  fail "suppressions without a reason: $(echo "$bare" | wc -l)"
else
  pass "suppressions: ${#supp[@]}, each with a reason"
fi

if (( skip_build )); then
  echo "SKIPPED compiler warnings (fresh build) and sanitizers (--skip-build)"
  exit "$rc"
fi

# 4. compiler warnings in a fresh build
fresh="$(python3 "$REEF_ROOT/scripts/colcon_tree.py")-fresh"
rm -rf "$fresh"
mkdir -p "$fresh"
echo "== fresh build of every package in ${fresh#"$REEF_ROOT"/} (warnings counted)"
if ! CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}" colcon --log-base "$fresh/log" build --base-paths src \
    --build-base "$fresh/build" --install-base "$fresh/install" --event-handlers console_direct+ >"$fresh/build.log" 2>&1; then
  tail -30 "$fresh/build.log"
  echo "BLOCKED: fresh build failed (${fresh#"$REEF_ROOT"/}/build.log)"
  exit 2
fi
own="$(grep -E ': warning: ' "$fresh/build.log" | grep -v '/src/third_party/' || true)"
all="$(grep -cE ': warning: ' "$fresh/build.log" || true)"
if [[ -n "$own" ]]; then
  echo "$own" | head -30
  fail "compiler warnings in the project's sources: $(echo "$own" | wc -l) (all warnings: $all)"
else
  pass "compiler warnings: 0 in the project's sources (fresh build of $(grep -c '^Finished <<<' "$fresh/build.log") packages; all warnings: $all)"
fi

# 5. sanitizers
if "$REEF_ROOT/scripts/check_sanitizers.sh" >"$fresh/sanitizers.log" 2>&1; then
  pass "sanitizers: $(tail -1 "$fresh/sanitizers.log" | sed 's/^PASS sanitizer profile: //')"
else
  s=$?
  tail -12 "$fresh/sanitizers.log"
  (( s == 2 )) && { echo "BLOCKED: sanitizer build could not run"; exit 2; }
  fail "sanitizers (${fresh#"$REEF_ROOT"/}/sanitizers.log)"
fi
exit "$rc"
