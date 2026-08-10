#!/usr/bin/env bash
# Record one explorer run as four synchronized views: belief, truth, Gazebo,
# RViz.
#
# Gazebo and RViz are real windows, but they cannot simply be screen-grabbed on
# WSLg: it is rootless, so each app is its own Windows window and the X root is
# empty (a grab returns black). Each app therefore gets a nested Xvfb screen
# with a tiny window manager to maximise it, and ffmpeg captures that screen's
# root -- which works headless anywhere, WSL or CI.
#
# Rendering is llvmpipe (software). That is why the maze is small and the
# capture rate is low; a 16x16 run under software Gazebo takes far too long.
#
#   ./scripts/record_quad_session.sh [--size 8] [--seed 5] [--fps 4]
# No `set -u`: the ROS setup scripts read unbound variables by design.
set -o pipefail

# EVERY thins the output: capture is wall-clock, so a run that takes minutes
# would otherwise produce a GIF of many hundreds of frames.
SIZE=8; SEED=5; FPS=4; EVERY=6; OUT=media/quad.gif
while [ $# -gt 0 ]; do
  case "$1" in
    --size)  SIZE=$2;  shift 2;;
    --seed)  SEED=$2;  shift 2;;
    --fps)   FPS=$2;   shift 2;;
    --every) EVERY=$2; shift 2;;
    --out)   OUT=$2;   shift 2;;
    *) echo "unknown argument: $1"; exit 2;;
  esac
done

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK=/tmp/quad
GZ_DISPLAY=:91
RVIZ_DISPLAY=:92
SCREEN=760x760
MAZE=$HOME/.micromouse_quad

cleanup() {
  pkill -9 -f "ffmpeg -loglevel error -f x11grab" 2>/dev/null
  pkill -9 -f "gz sim"        2>/dev/null
  pkill -9 -f rviz2           2>/dev/null
  pkill -9 -f raycast_lidar   2>/dev/null
  pkill -9 -f cell_motion_controller 2>/dev/null
  pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f matchbox-window 2>/dev/null
  pkill -9 -f "Xvfb $GZ_DISPLAY"   2>/dev/null
  pkill -9 -f "Xvfb $RVIZ_DISPLAY" 2>/dev/null
}
trap cleanup EXIT
cleanup; sleep 2

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1        # llvmpipe; no GPU on this host
rm -rf "$WORK"; mkdir -p "$WORK"

echo "== generating a ${SIZE}x${SIZE} maze (seed $SEED) =="
( cd "$WS/src/micromouse" && python3 -m micromouse.generate_maze \
    --size "$SIZE" --seed "$SEED" --top-down-cam --out-dir "$MAZE" >/dev/null )
export MICROMOUSE_SPEC=$MAZE/maze_spec.json

start_display() {                      # $1 = display, $2 = geometry
  Xvfb "$1" -screen 0 "${2}x24" -nolisten tcp >/dev/null 2>&1 &
  sleep 2
  DISPLAY=$1 matchbox-window-manager -use_titlebar no >/dev/null 2>&1 &
  sleep 1
}
start_display "$GZ_DISPLAY"   "$SCREEN"
start_display "$RVIZ_DISPLAY" "$SCREEN"

echo "== simulation (physics headless; the GUI gets its own screen) =="
ros2 launch micromouse maze_gazebo.launch.py \
  headless:=true viz:=false motion:=velocity lidar:=raycast \
  spec:=$MAZE/maze_spec.json truth:=$MAZE/maze_truth.json \
  world:=$MAZE/maze.world > "$WORK/launch.log" 2>&1 &
sleep 25

DISPLAY=$GZ_DISPLAY gz sim -g "$MAZE/maze.world" > "$WORK/gzgui.log" 2>&1 &
DISPLAY=$RVIZ_DISPLAY rviz2 -d "$WS/src/micromouse/config/explorer.rviz" \
  > "$WORK/rviz.log" 2>&1 &
echo "== waiting 75s for software rendering to settle =="
sleep 75
DISPLAY=$GZ_DISPLAY   xdotool search --sync --onlyvisible --class . \
  windowsize 100% 100% >/dev/null 2>&1
DISPLAY=$RVIZ_DISPLAY xdotool search --sync --onlyvisible --class . \
  windowsize 100% 100% >/dev/null 2>&1

echo "== capturing =="
ffmpeg -loglevel error -f x11grab -framerate "$FPS" -video_size "$SCREEN" \
  -i "$GZ_DISPLAY" -pix_fmt yuv420p -y "$WORK/gz.mkv" &
ffmpeg -loglevel error -f x11grab -framerate "$FPS" -video_size "$SCREEN" \
  -i "$RVIZ_DISPLAY" -pix_fmt yuv420p -y "$WORK/rviz.mkv" &
sleep 2

cd "$WS"
python3 scripts/record_quad_gif.py collect --dir "$WORK" &
COLLECT=$!
sleep 2

echo "== running the explorer (closed-loop, real wheels) =="
timeout 1800 python3 scripts/run_explorer_headless.py --attach 2>&1 \
  | grep -E 'run complete|search run|return trip|speed run|optimal'

sleep 4
pkill -INT -f "ffmpeg -loglevel error -f x11grab"
wait $COLLECT
sleep 2

echo "== composing =="
python3 scripts/record_quad_gif.py compose --dir "$WORK" \
  --gz "$WORK/gz.mkv" --rviz "$WORK/rviz.mkv" --fps "$FPS" \
  --every "$EVERY" --out "$OUT"
