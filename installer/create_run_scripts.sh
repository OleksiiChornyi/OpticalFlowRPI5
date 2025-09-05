#!/bin/bash
set -e
echo "===================="
echo "5. Создание скриптов запуска"
echo "===================="

mkdir -p ~/autorun

# 5.1 MAVROS
cat > ~/autorun/start_mavros.sh << 'EOF'
#!/bin/bash
while [ ! -e /dev/ttyACM0 ]; do
    echo "/dev/ttyACM0 не найден, ждем..."
    sleep 2
done
echo "Запускаем MAVROS..."
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 fcu_protocol:="v2.0"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
EOF
chmod +x ~/autorun/start_mavros.sh

# 5.2 Optical flow
cat > ~/autorun/start_optical_flow.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
ros2 run optical_flow optical_flow
EOF
chmod +x ~/autorun/start_optical_flow.sh

# 5.3 TMUX запуск всех
cat > ~/autorun/start_all_tmux.sh << 'EOF'
#!/bin/bash
SESSION="optical_flow_session"
if tmux has-session -t $SESSION 2>/dev/null; then
    tmux kill-session -t $SESSION
fi
tmux new-session -d -s $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/start_mavros.sh" C-m
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/start_optical_flow.sh" C-m
EOF
chmod +x ~/autorun/start_all_tmux.sh

echo "Скрипты запуска созданы!"
