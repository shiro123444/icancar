#!/bin/bash
# Own only this trial and its serial adapter; restore the normal read-only base.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ -e "$ROOT/runtime/OPERATOR_HOLD" ]]; then
    echo '用户已暂停实车测试；仅允许只读采集。'
    exit 3
fi
MODE="${1:-lap}"
ID="${2:?session id required}"
[[ "$ID" =~ ^[a-zA-Z0-9-]+$ ]] || exit 2
case "$MODE" in lap) SECONDS_LIMIT=150; EXTRA=(--start-landmark 1);; crosswalk) SECONDS_LIMIT=15; EXTRA=(--stop-after-crosswalk);; *) exit 2;; esac
OUTPUT="$ROOT/logs/supervised-$ID"
STATE="$ROOT/runtime"
STOP="$STATE/supervised-$ID.stop"
[[ ! -e "$OUTPUT" && ! -e "$STOP" ]] || exit 2
base_stopped=0
driver_pid=''
trial_pid=''
cleanup(){
    set +e
    if [[ -n "$trial_pid" ]] && kill -0 "$trial_pid" 2>/dev/null; then
        touch "$STOP"
        kill -INT "$trial_pid" 2>/dev/null
        for _ in {1..30}; do kill -0 "$trial_pid" 2>/dev/null || break; sleep .1; done
        kill -TERM "$trial_pid" 2>/dev/null
    fi
    if [[ -n "$driver_pid" ]] && kill -0 "$driver_pid" 2>/dev/null; then
        kill -INT "$driver_pid"
        for _ in {1..30}; do kill -0 "$driver_pid" 2>/dev/null || break; sleep .1; done
        kill -TERM "$driver_pid" 2>/dev/null
    fi
    if [[ "$base_stopped" == 1 ]] && ! fuser /dev/deepbotbase >/dev/null 2>&1; then
        bash ros2/run_foxy.sh python3 - <<'PY'
import serial,time
from automation.base_protocol import velocity_frame
with serial.Serial('/dev/deepbotbase',115200,timeout=0,write_timeout=.2,exclusive=True) as p:
 for _ in range(3):p.write(velocity_frame(0.,0.,0.));time.sleep(.03)
PY
        nohup bash ros2/run_foxy.sh python3 ros2/deepbot_base.py --device /dev/deepbotbase --publish-hz 20 --sensor-topic /visual_race/base_sensor_diagnostics >runtime/base.log 2>&1 < /dev/null &
        printf '%s\n' "$!" >runtime/base.pid
    fi
    sync
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 - <<'PY'
from pathlib import Path
import os,signal,time
root=Path.cwd();p=root/'runtime/base.pid';pid=int(p.read_text().strip())
assert Path('/proc/%d/cwd'%pid).resolve()==root
args=Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0')
assert b'ros2/deepbot_base.py' in args and b'--enable-drive' not in args
os.kill(pid,signal.SIGTERM)
for _ in range(30):
 if not Path('/proc/%d'%pid).exists():break
 time.sleep(.1)
else:raise SystemExit('owned base reader did not exit')
p.unlink()
PY
base_stopped=1
timeout 6 bash -c 'while fuser /dev/deepbotbase >/dev/null 2>&1; do sleep .2; done'
nohup bash ros2/run_foxy.sh python3 ros2/deepbot_base.py --device /dev/deepbotbase --enable-drive --max-linear .2 --max-angular .65 --cmd-topic /visual_race/supervised/cmd_vel --cmd-timeout .3 --cmd-max-age .3 --publish-hz 20 --sensor-topic /visual_race/base_sensor_diagnostics >"$STATE/supervised-$ID-base.log" 2>&1 < /dev/null &
driver_pid=$!
printf '%s\n' "$driver_pid" >"$STATE/supervised-$ID-base.pid"
nohup timeout -s INT -k 2s "$((SECONDS_LIMIT+18))s" bash ros2/run_foxy.sh python3 ros2/supervised_track_test.py --config data/perception.site-review.json --model-config data/crosswalk-model.site-review.json --output-dir "$OUTPUT" --seconds "$SECONDS_LIMIT" --speed .2 --operator-at-power-switch --run-motion-test --stop-file "$STOP" "${EXTRA[@]}" >"$STATE/supervised-$ID.log" 2>&1 < /dev/null &
trial_pid=$!
printf '%s\n' "$trial_pid" >"$STATE/supervised-$ID.pid"
set +e
wait "$trial_pid"
status=$?
cat "$STATE/supervised-$ID.log"
exit "$status"
