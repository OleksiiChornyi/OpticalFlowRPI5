#!/bin/bash
set -e
echo "===================="
echo "4. Создание run_xxx.sh скриптов"
echo "===================="

mkdir -p ~/autorun

# --- Запуск MAVROS ---
cat > ~/autorun/run_mavros.sh << 'EOF'
#!/bin/bash
echo "Запуск MAVROS..."
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 fcu_protocol:="v2.0"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
EOF
chmod +x ~/autorun/run_mavros.sh

# --- Запуск Optical Flow Node ---
cat > ~/autorun/run_optical_flow.sh << 'EOF'
#!/bin/bash
echo "Запуск Optical Flow Node..."
source ~/optical_flow_ws/install/setup.bash
ros2 run optical_flow_pkg optical_flow
EOF
chmod +x ~/autorun/run_optical_flow.sh

# --- Запуск всех через tmux ---
cat > ~/autorun/run_all_tmux.sh << 'EOF'
#!/bin/bash
SESSION="optical_flow_session"

if tmux has-session -t $SESSION 2>/dev/null; then
    tmux kill-session -t $SESSION
fi

tmux new-session -d -s $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/run_mavros.sh" C-m
tmux split-window -v
tmux send-keys -t $SESSION "bash ~/autorun/run_optical_flow.sh" C-m
tmux select-pane -t 0
tmux attach -t $SESSION
EOF
chmod +x ~/autorun/run_all_tmux.sh

echo "Скрипты запуска созданы!"
