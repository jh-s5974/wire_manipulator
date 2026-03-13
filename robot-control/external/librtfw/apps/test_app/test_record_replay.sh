#!/bin/bash
# test_app_record_replay.sh
#
# End-to-end record/replay integrity test for test_app.
#
# What is verified:
#   A) Structural checks – file exists, rtcli check_record passes, archive keys present
#   B) Derived-value reproduction:
#        RT-origin pure fn    → must match
#        RT-origin tick fn    → must match (tick is restored in replay)
#        NonRT-origin pure fn → must match
#        NonRT-origin tick fn → must match (tick is restored in replay)
#
# Key design:
#   - WARM-UP is 8 s before recording starts.
#   - DerivedCheck_Logger logs all 4 derived keys at 5 Hz (NonRT).
#   - The pure-function checks verify that non-archived keys which depend
#     only on archived data reproduce correctly during replay.
#   - Tick-dependent chains are expected to match because checkpoint now
#     restores framework/timeline tick context used by task execution.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"
APP="$BUILD_DIR/apps/test_app/test_app"
RTCLI="$BUILD_DIR/tools/rtcli"
RECORD_FILE="/tmp/test_app_stress.rtrec"
APP_LOG_LIVE="/tmp/test_app_live.log"
APP_LOG_REPLAY="/tmp/test_app_replay.log"

RECORD_START_ISO=""   # diagnostic wall-clock only
REPLAY_START_ISO=""   # diagnostic wall-clock only

PASS=0
FAIL=0

# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
check() {
    local desc="$1" cond="$2"
    if eval "$cond"; then
        printf "  ✅  PASS  %s\n" "$desc"; ((PASS++)) || true
    else
        printf "  ❌  FAIL  %s\n" "$desc"; ((FAIL++)) || true
    fi
}

check_val() {
    # check_val "desc" <result: 0=pass 1=fail>
    local desc="$1" rc="$2"
    if [[ ! "$rc" =~ ^[0-9]+$ ]]; then
        rc=1
    fi
    if [[ "$rc" -eq 0 ]]; then
        printf "  ✅  PASS  %s\n" "$desc"; ((PASS++)) || true
    else
        printf "  ❌  FAIL  %s\n" "$desc"; ((FAIL++)) || true
    fi
}

cleanup() {
    pkill -f "build/apps/test_app/test_app" 2>/dev/null || true
    sleep 0.5
}

require_binary() {
    if [[ ! -x "$1" ]]; then
        echo "ERROR: '$1' not found – build first: cd build && cmake --build . --target test_app rtcli"
        exit 1
    fi
}

find_marker_line() {
    local logfile="$1" pattern="$2"
    local line
    line=$(grep -n "$pattern" "$logfile" | tail -1 | cut -d: -f1 || true)
    if [[ -z "${line:-}" ]]; then
        echo 0
    else
        echo "$line"
    fi
}

# Extract [DerivedCheck] lines after a marker line number.
# Args: <logfile> <marker_line_no>
# Prints: "g_tick rt_pure rt_tick nonrt_pure nonrt_tick" per line
extract_derived_after_line() {
    local logfile="$1" marker_line="$2"
    awk -v marker_line="$marker_line" '
        NR > marker_line && /\[DerivedCheck\]/ {
            gt="-1"
            rp="0"; rt="0"; nrp="0"; nrt="0"
            for (i=1; i<=NF; i++) {
                if ($i ~ /^g_tick=/)     { split($i,a,"="); gt=a[2] }
                if ($i ~ /^rt_pure=/)    { split($i,a,"="); rp=a[2] }
                if ($i ~ /^rt_tick=/)    { split($i,a,"="); rt=a[2] }
                if ($i ~ /^nonrt_pure=/) { split($i,a,"="); nrp=a[2] }
                if ($i ~ /^nonrt_tick=/) { split($i,a,"="); nrt=a[2] }
            }
            if (gt != "-1") {
                print gt, rp, rt, nrp, nrt
            }
        }
    ' "$logfile"
}

# ─────────────────────────────────────────────
# Pre-flight
# ─────────────────────────────────────────────
echo "========================================"
echo "  test_app  record/replay stress test"
echo "========================================"
echo ""
require_binary "$APP"
require_binary "$RTCLI"
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 required for data comparison"; exit 1; }

cleanup
rm -f "$RECORD_FILE" "$APP_LOG_LIVE" "$APP_LOG_REPLAY"

# ─────────────────────────────────────────────
# Phase 1 – LIVE warm-up (8 s) + record (10 s)
# ─────────────────────────────────────────────
echo "[Phase 1]  Start test_app LIVE – warm-up 8 s, then record 10 s"
echo "           (8 s warm-up ensures statistics have real accumulated state)"
"$APP" > "$APP_LOG_LIVE" 2>&1 &
APP_PID=$!
echo "  app PID = $APP_PID"

echo "  Warm-up 8 s..."
sleep 8

echo "  Starting record → $RECORD_FILE"
"$RTCLI" record start "$RECORD_FILE"
RECORD_START_ISO=$(date '+%Y-%m-%d %H:%M:%S.%3N')
echo "  Record command accepted (wall-clock: $RECORD_START_ISO)"

echo "  Recording 10 s..."
sleep 10

"$RTCLI" record stop
echo "  Record stopped."

sleep 1
kill "$APP_PID" 2>/dev/null || true
wait "$APP_PID" 2>/dev/null || true
echo "  App stopped."
echo ""

# ─────────────────────────────────────────────
# Phase 2 – Structural record validation
# ─────────────────────────────────────────────
echo "[Phase 2]  Structural validation of record file"
CHECK_OUT=$("$RTCLI" check_record "$RECORD_FILE" 2>&1)
echo "$CHECK_OUT" | head -30
echo ""

check "Record file exists"           "[[ -f '$RECORD_FILE' ]]"
check "check_record exits 0"         "'$RTCLI' check_record '$RECORD_FILE' >/dev/null 2>&1"

KEY_COUNT=$(echo "$CHECK_OUT" | grep -c \
    "stat\.10\.summary\|raw\.1k\.\|filt\.1k\.\|fused\.500\.\|cmd\.1k\.ctrl\|mon\.100\.\|stat\.10\.\|stress\.50\.d\|param\.gain" \
    2>/dev/null || echo "0")
check "≥8 archive keys in record"    "[[ $KEY_COUNT -ge 8 ]]"

check "CHECKPOINT entry present"     "echo '$CHECK_OUT' | grep -q '<CHECKPOINT>'"

LIVE_SUMMARY_LINES=$(grep -c '\[Summary\]' "$APP_LOG_LIVE" 2>/dev/null || echo 0)
check "Live app produced Summary log lines (≥5)"  "[[ $LIVE_SUMMARY_LINES -ge 5 ]]"
check "No FATAL in live log"         "! grep -q 'FATAL\|fatal' '$APP_LOG_LIVE'"
echo ""

# ─────────────────────────────────────────────
# Phase 3 – Replay (fresh app, replay starts within 3 s)
# ─────────────────────────────────────────────
echo "[Phase 3]  Replay – fresh app, replay starts within 3 s of init"
echo "           (if checkpoint is NOT restored, statistics start near-zero"
echo "            and will diverge from the live values at record-start)"
"$APP" > "$APP_LOG_REPLAY" 2>&1 &
APP_PID=$!
echo "  app PID = $APP_PID"

echo "  Waiting 3 s for init..."
sleep 3

echo "  Starting replay"
"$RTCLI" replay start "$RECORD_FILE"
REPLAY_START_ISO=$(date '+%Y-%m-%d %H:%M:%S.%3N')
echo "  Replay command accepted (wall-clock: $REPLAY_START_ISO)"

echo "  Replay running 12 s..."
sleep 12

"$RTCLI" replay stop
echo "  Replay stopped."

sleep 1
kill "$APP_PID" 2>/dev/null || true
wait "$APP_PID" 2>/dev/null || true
echo "  App stopped."
echo ""

# ─────────────────────────────────────────────
# Phase 4 – Structural replay validation
# ─────────────────────────────────────────────
echo "[Phase 4]  Structural replay validation"

check "Replay log exists (non-empty)"         "[[ -s '$APP_LOG_REPLAY' ]]"
check "No FATAL in replay log"                "! grep -q 'FATAL\|fatal' '$APP_LOG_REPLAY'"
REPLAY_SUMMARY_LINES=$(grep -c '\[Summary\]' "$APP_LOG_REPLAY" 2>/dev/null || echo 0)
check "Replay produced Summary lines (≥5)"    "[[ $REPLAY_SUMMARY_LINES -ge 5 ]]"
check "StatsLog_A active during replay (≥1)"  "grep -q '\[StatsLog_A\]' '$APP_LOG_REPLAY'"
echo ""

# ─────────────────────────────────────────────
# Phase 6 – Derived-value reproduction check
# ─────────────────────────────────────────────
# Four non-archived keys are compared between live (record window) and replay:
#
#   derived.rt.pure    = pure f(filt.1k.a, cmd.1k.ctrl)  NO tick
#                        Both inputs are archived (RT origin).
#                        → During replay: inputs injected from file.
#                          Pure function → output MUST match.
#
#   derived.rt.tick    = f(filt.1k.a) * (1 + tick * 0.01)
#                        Same archived input + getExecutionLocalTick().
#                        → Replay restores tick context, output MUST match.
#
#   derived.nonrt.pure = gain² + offset        pure f(param.gain.a, archived NonRT)
#                        → Replay injected → output MUST match.
#
#   derived.nonrt.tick = gain * (1 + tick * 0.1)
#                        NonRT tick context is restored too.
#                        → output MUST match.
#
# Test PASS criteria:
#   all chains  → max_diff < TOLERANCE_MATCH
# ─────────────────────────────────────────────
echo "[Phase 5]  Derived-value reproduction: pure-function chains vs tick-tainted chains"
echo ""
echo "  Keys verified (all non-archived):"
echo "    derived.rt.pure     pure f(archived RT)        → MUST match"
echo "    derived.rt.tick     f(archived RT) + tick       → MUST match"
echo "    derived.nonrt.pure  pure f(archived NonRT)      → MUST match"
echo "    derived.nonrt.tick  f(archived NonRT) * tick    → MUST match"
echo ""

LIVE_MARKER_LINE=$(find_marker_line "$APP_LOG_LIVE" "\\[Action\\] recording started: .*marker_tick=")
REPLAY_MARKER_LINE=$(find_marker_line "$APP_LOG_REPLAY" "\\[Action\\] replay started: .*marker_tick=")

check "Live marker line found"   "[[ $LIVE_MARKER_LINE -gt 0 ]]"
check "Replay marker line found" "[[ $REPLAY_MARKER_LINE -gt 0 ]]"

LIVE_DERIVED=$(  extract_derived_after_line "$APP_LOG_LIVE"   "$LIVE_MARKER_LINE")
REPLAY_DERIVED=$(extract_derived_after_line "$APP_LOG_REPLAY" "$REPLAY_MARKER_LINE")

LIVE_N=$(  echo "$LIVE_DERIVED"   | grep -c . 2>/dev/null || echo 0)
REPLAY_N=$(echo "$REPLAY_DERIVED" | grep -c . 2>/dev/null || echo 0)
echo "  Live marker line                             : $LIVE_MARKER_LINE"
echo "  Replay marker line                           : $REPLAY_MARKER_LINE"
echo "  Live   [DerivedCheck] lines after marker    : $LIVE_N"
echo "  Replay [DerivedCheck] lines after marker    : $REPLAY_N"
echo ""
echo "  Live first 3 lines:"
echo "$LIVE_DERIVED"   | head -3 | awk '{printf "    g_tick=%-8s rt_pure=%-14s rt_tick=%-14s nonrt_pure=%-14s nonrt_tick=%s\n",$1,$2,$3,$4,$5}'
echo "  Replay first 3 lines:"
echo "$REPLAY_DERIVED" | head -3 | awk '{printf "    g_tick=%-8s rt_pure=%-14s rt_tick=%-14s nonrt_pure=%-14s nonrt_tick=%s\n",$1,$2,$3,$4,$5}'
echo ""

set +e
DERIVED_RESULT=$(python3 - "$LIVE_DERIVED" "$REPLAY_DERIVED" << 'PYEOF'
import sys

def parse_series(text):
    rows = []
    for line in text.strip().splitlines():
        parts = line.split()
        if len(parts) == 5:
            try:
                gt = int(parts[0])
                vals = tuple(float(p) for p in parts[1:])
                rows.append((gt, *vals))
            except ValueError:
                pass
    return rows  # each row: (g_tick, rt_pure, rt_tick, nonrt_pure, nonrt_tick)

live   = parse_series(sys.argv[1])   # ground truth: values from record window
replay = parse_series(sys.argv[2])   # replay output: must contain live sequence

MIN_PAIRS   = 5      # minimum pairs needed for a valid comparison
TOL_MATCH   = 1e-4   # max allowed diff (double precision, exact replay)

print(f"  live_lines={len(live)}  replay_lines={len(replay)}")

if len(live) < MIN_PAIRS or len(replay) < MIN_PAIRS:
    print(f"INSUFFICIENT_DATA  need>={MIN_PAIRS} in both")
    sys.exit(2)

# ── Tick-keyed strict alignment ────────────────────────────────────────────
live_map = {row[0]: row[1:] for row in live}
replay_map = {row[0]: row[1:] for row in replay}
common_ticks = sorted(set(live_map.keys()) & set(replay_map.keys()))
n_eval = len(common_ticks)

print(f"ALIGNMENT  mode=tick-keyed  common_ticks={n_eval}")

if n_eval < MIN_PAIRS:
    print(f"INSUFFICIENT_DATA  common_ticks<{MIN_PAIRS}")
    sys.exit(2)

live_eval = [live_map[t] for t in common_ticks]
replay_eval = [replay_map[t] for t in common_ticks]

# ── Per-channel checks ─────────────────────────────────────────────────────
# col 0: rt_pure    – pure f(archived RT)   → must match
# col 1: rt_tick    – f(archived RT)+tick   → must diverge
# col 2: nonrt_pure – pure f(archived NonRT)→ must match
# col 3: nonrt_tick – f(archived NonRT)+tick→ must diverge
checks = [
    ("RT_pure_match",         0, "max<", TOL_MATCH),
    ("RT_tick_match",         1, "max<", TOL_MATCH),
    ("NonRT_pure_match",      2, "max<", TOL_MATCH),
    ("NonRT_tick_match",      3, "max<", TOL_MATCH),
]

all_ok = True
for label, col, mode, thr in checks:
    diffs  = [abs(live_eval[i][col] - replay_eval[i][col]) for i in range(n_eval)]
    metric = max(diffs) if mode == "max<" else sum(diffs) / len(diffs)
    ok     = (metric < thr) if mode == "max<" else (metric > thr)
    status = "PASS" if ok else "FAIL"
    verb   = "max_diff" if mode == "max<" else "avg_diff"
    print(f"  {status}  {label:<26} n={n_eval}  {verb}={metric:.8f}  thr={thr}")
    if not ok:
        all_ok = False
        worst = max(range(n_eval), key=lambda i: abs(live_eval[i][col] - replay_eval[i][col]))
        print(f"         worst step={worst}  "
              f"tick={common_ticks[worst]}  live={live_eval[worst][col]:.8f}  replay={replay_eval[worst][col]:.8f}")

# ── Visual: aligned first 5 pairs ─────────────────────────────────────────
print(f"\n  First 5 aligned pairs  (tick-keyed):")
for i in range(min(5, n_eval)):
    match = "==" if abs(live_eval[i][0] - replay_eval[i][0]) < TOL_MATCH else "!!"
    print(f"    [{i}] tick={common_ticks[i]} {match} live  rt_pure={live_eval[i][0]:+.8f}  nonrt_pure={live_eval[i][2]:+.8f}")
    print(f"           rply  rt_pure={replay_eval[i][0]:+.8f}  nonrt_pure={replay_eval[i][2]:+.8f}")

sys.exit(0 if all_ok else 1)
PYEOF
)
DERIVED_RC=$?
set -e

echo "$DERIVED_RESULT"
echo ""

if echo "$DERIVED_RESULT" | grep -q "INSUFFICIENT_DATA"; then
    check_val "Enough [DerivedCheck] pairs for comparison" 1
else
    RT_PURE_OK=$( echo "$DERIVED_RESULT" | awk '$2=="RT_pure_match"{print ($1=="PASS") ? 0 : 1}')
    RT_TICK_OK=$( echo "$DERIVED_RESULT" | awk '$2=="RT_tick_match"{print ($1=="PASS") ? 0 : 1}')
    NRT_PURE_OK=$(echo "$DERIVED_RESULT" | awk '$2=="NonRT_pure_match"{print ($1=="PASS") ? 0 : 1}')
    NRT_TICK_OK=$(echo "$DERIVED_RESULT" | awk '$2=="NonRT_tick_match"{print ($1=="PASS") ? 0 : 1}')

    check_val "derived.rt.pure    (archived RT input, pure fn)   → matches replay"    "${RT_PURE_OK:-1}"
    check_val "derived.rt.tick    (archived RT input + tick)      → matches replay" "${RT_TICK_OK:-1}"
    check_val "derived.nonrt.pure (archived NonRT input, pure fn) → matches replay"   "${NRT_PURE_OK:-1}"
    check_val "derived.nonrt.tick (archived NonRT input + tick)   → matches replay" "${NRT_TICK_OK:-1}"
fi

if [[ $DERIVED_RC -ne 0 ]]; then
    echo "  (info) Derived comparison returned rc=$DERIVED_RC; checks above reflect pass/fail details."
fi
echo ""

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────
TOTAL=$((PASS + FAIL))
echo "========================================"
printf "  Results: %d / %d passed\n" "$PASS" "$TOTAL"
echo "========================================"
echo ""
if [[ $FAIL -eq 0 ]]; then
    echo "  All checks passed."
    echo "  Logs: $APP_LOG_LIVE  $APP_LOG_REPLAY"
    exit 0
else
    echo "  $FAIL check(s) failed."
    echo "  Logs: $APP_LOG_LIVE  $APP_LOG_REPLAY"
    exit 1
fi
