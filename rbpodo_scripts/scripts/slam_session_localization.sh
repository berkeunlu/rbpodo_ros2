#!/usr/bin/env bash
# Start a tmux session with ZED camera (left) and RTAB-Map (right).
# Usage: ./slam_session.sh [session_name]

SESSION="${1:-rbpodo_slam}"
WS="$HOME/ros2_ws"
SETUP="source $WS/install/setup.bash"

# Kill existing session if present
tmux kill-session -t "$SESSION" 2>/dev/null

tmux new-session -d -s "$SESSION" -x 220 -y 50

# Left pane: ZED camera
tmux send-keys -t "$SESSION:0.0" "$SETUP && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zedm" Enter

# Right pane: RTAB-Map (wait 5s for ZED to init)
tmux split-window -h -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.1" "sleep 5 && $SETUP && ros2 launch rbpodo_slam rtabmap_zedm.launch.py delete_db:=false localization:=true" Enter

tmux select-pane -t "$SESSION:0.0"
echo "All tmux panes in session '${SESSION}' are up and running."
tmux attach-session -t "$SESSION"
