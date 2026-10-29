#!/bin/bash
# Start/stop only the read-only DEEPBOT camera, odometry and task preview.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STATE="$ROOT/runtime"
RUNNER="$ROOT/ros2/run_foxy.sh"
CAMERA=/dev/video0
BASE=/dev/deepbotbase
VOICE=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84037923-if00
CONFIG="${VISION_PERCEPTION_CONFIG:-$ROOT/data/perception.lab-example.json}"
MODEL_CONFIG="${VISION_MODEL_CONFIG:-$ROOT/data/crosswalk-model.example.json}"
BARRIER_CONFIG="$ROOT/data/barrier.example.json"
MODEL="$HOME/catkin_ws/src/deepbot_project/deepbot_openvino/model/race.xml"
STEERING_MODEL="$ROOT/data/models/steering.pth"
mkdir -p "$STATE"
cd "$ROOT"

expected_script(){
  case "$1" in
    camera) echo 'ros2/camera_node.py';;
    base) echo 'ros2/deepbot_base.py';;
    voice) echo 'ros2/serial_voice_player.py';;
    model) echo 'ros2/openvino_preview.py';;
    steering) echo 'ros2/steering_preview.py';;
    barrier) echo 'ros2/barrier_detector.py';;
    mission) echo 'ros2/automatic_race.py';;
    bridge) echo 'ros2/telemetry_bridge.py';;
    *) return 1;;
  esac
}
is_ours(){
  local role="$1" pid expected cwd command
  [[ -f "$STATE/$role.pid" ]] || return 1
  read -r pid < "$STATE/$role.pid"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  expected="$(expected_script "$role")"
  cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
  command="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ "$cwd" == "$ROOT" && "$command" == *"$expected"* ]]
}
stop_ours(){
  local role pid
  for role in bridge mission barrier steering model voice base camera; do
    if is_ours "$role"; then
      read -r pid < "$STATE/$role.pid"
      kill -TERM "$pid"
      echo "$role 已停止"
    fi
    rm -f "$STATE/$role.pid"
  done
}
start_one(){
  local role="$1"
  shift
  nohup bash "$RUNNER" python3 "$@" > "$STATE/$role.log" 2>&1 < /dev/null &
  echo "$!" > "$STATE/$role.pid"
}
case "${1:-status}" in
  status)
    for role in camera base voice mission model steering barrier bridge; do
      if is_ours "$role"; then echo "$role 运行中"; else echo "$role 未运行"; fi
    done
    ;;
  stop) stop_ours;;
  start)
    for role in camera base voice mission model steering barrier bridge; do
      if is_ours "$role"; then echo '预览程序已经运行，请先查看状态。'; exit 1; fi
    done
    [[ -f /opt/ros/foxy/setup.bash && -f "$CONFIG" && -f "$MODEL_CONFIG" ]] || { echo 'ROS 2 或识别配置未找到。'; exit 1; }
    for device in "$CAMERA" "$BASE" "$VOICE"; do
      [[ -e "$device" ]] || { echo "设备不存在: $device"; exit 1; }
      if fuser "$device" >/dev/null 2>&1; then echo "设备正在使用: $device"; exit 1; fi
    done
    if ss -ltn | grep -qE '127\.0\.0\.1:8770[[:space:]]'; then echo '8770 端口已被使用。'; exit 1; fi
    CAMERA_ARGS=(--device "$CAMERA" --fps 10)
    if [[ -f "$ROOT/data/camera-intrinsics.yaml" ]]; then CAMERA_ARGS+=(--calibration "$ROOT/data/camera-intrinsics.yaml"); fi
    start_one camera ros2/camera_node.py "${CAMERA_ARGS[@]}"
    start_one base ros2/deepbot_base.py --device "$BASE" --publish-hz 20 --sensor-topic /visual_race/base_sensor_diagnostics
    start_one voice ros2/serial_voice_player.py --device "$VOICE"
    start_one mission ros2/automatic_race.py --camera-topic /camera/rgb/image_raw/compressed --camera-type compressed --odom-topic /raw_odom --config "$CONFIG" --crosswalk-source model --model-config "$MODEL_CONFIG" --events-dir logs/ros2-events
    if [[ -f "$MODEL" ]]; then start_one model ros2/openvino_preview.py --model "$MODEL" --model-config "$MODEL_CONFIG" --max-fps 5; fi
    if [[ -f "$STEERING_MODEL" && -f "$ROOT/training/train_steering.py" ]]; then start_one steering ros2/steering_preview.py --model "$STEERING_MODEL"; fi
    if [[ -f "$BARRIER_CONFIG" ]]; then start_one barrier ros2/barrier_detector.py --config "$BARRIER_CONFIG"; fi
    start_one bridge ros2/telemetry_bridge.py --port 8770 --camera-topic /camera/rgb/image_raw/compressed --camera-type compressed --odom-topic /raw_odom --mission-topic /visual_race/status --model-topic /visual_race/model_detections --calibration-topic /visual_race/calibration --steering-topic /visual_race/steering_preview --control-topic /visual_race/control_diagnostics --barrier-topic /visual_race/barrier_observation --avoidance-topic /visual_race/avoidance_diagnostics
    sleep 3
    for role in camera base voice mission bridge; do
      if ! is_ours "$role"; then
        echo "$role 启动失败，查看 $STATE/$role.log"
        stop_ours
        exit 1
      fi
    done
    if [[ -f "$MODEL" ]] && ! is_ours model; then echo '模型预览未启动，查看 runtime/model.log'; fi
    if [[ -f "$STEERING_MODEL" ]] && ! is_ours steering; then echo '循迹预览未启动，查看 runtime/steering.log'; fi
    if [[ -f "$BARRIER_CONFIG" ]] && ! is_ours barrier; then echo '挡板预览未启动，查看 runtime/barrier.log'; fi
    echo '只读预览已启动。'
    ;;
  *) echo "用法: bash $0 {start|stop|status}"; exit 2;;
esac
