#!/usr/bin/env bash
# Tmux session: rb10_1300e fake hardware + ZED Mini + RTAB-Map (localization).
#
# Layout:
#   ┌──────────────────────┬──────────────────────────┐
#   │  [0] Robot bringup   │                          │
#   ├──────────────────────┤  [1] RTAB-Map            │
#   │  [2] ZED camera      │      (localization)      │
#   └──────────────────────┴──────────────────────────┘
#
# Usage:
#   ./slam_session_localization_sim.sh [odom_source] [session_name]
#
# odom_source:
#   rtabmap (default) — rtabmap visual VO, TF: map->odom->link0->...->zedm_camera_link
#                       ZED publish_tf:=false (no TF loop)
#   zed               — ZED VIO (IMU-fused), TF: map->odom->zedm_camera_link
#                       ZED publish_tf:=true

ODOM_SOURCE="${1:-rtabmap}"
SESSION="${2:-rbpodo_slam_sim}"
WS="$HOME/ros2_ws"
SETUP="source $WS/install/setup.bash"

if [ "$ODOM_SOURCE" = "zed" ]; then
  ZED_TF_ARGS=""
else
  # rtabmap publishes odom->link0->...->zedm_camera_link via FK.
  # ZED must NOT publish odom->zedm_camera_link or it creates a TF loop.
  ZED_TF_ARGS="publish_tf:=false publish_map_tf:=false"
fi

tmux kill-session -t "$SESSION" 2>/dev/null
tmux new-session -d -s "$SESSION" -x 240 -y 60

# Step 1: split full window into left | right
tmux split-window -h -t "$SESSION:0.0"

# Step 2: split left pane vertically → top-left (0) and bottom-left (2)
tmux split-window -v -t "$SESSION:0.0"

# Pane 0 (top-left): robot bringup with fake hardware
tmux send-keys -t "$SESSION:0.0" \
  "$SETUP && ros2 launch rbpodo_bringup rbpodo.launch.py \
  model_id:=rb10_1300e use_fake_hardware:=true" Enter

# Pane 2 (bottom-left): ZED camera
tmux send-keys -t "$SESSION:0.2" \
  "sleep 5 && $SETUP && ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm camera_name:=zedm ${ZED_TF_ARGS}" Enter

# Pane 1 (right): RTAB-Map localization
tmux send-keys -t "$SESSION:0.1" \
  "sleep 10 && $SETUP && ros2 launch rbpodo_slam rtabmap_zedm.launch.py \
  odom_source:=${ODOM_SOURCE} localization:=true delete_db:=false" Enter

tmux select-pane -t "$SESSION:0.0"
echo "Session '${SESSION}' started (fake hardware, odom_source=${ODOM_SOURCE})."
echo "Pane 0: robot bringup | Pane 1: RTAB-Map | Pane 2: ZED camera"
tmux attach-session -t "$SESSION"
