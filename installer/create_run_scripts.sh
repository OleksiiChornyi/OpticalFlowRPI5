#!/bin/bash
set -e
echo "=== Создание скриптов запуска ==="

mkdir -p ~/rpi_ws/src/optical_flow_pkg

# run_camera.sh
cat > ~/rpi_ws/src/optical_flow_pkg/run_camera.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=1152 -p height:=864
EOF
chmod +x ~/rpi_ws/src/optical_flow_pkg/run_camera.sh

# run_mavros.sh
cat > ~/rpi_ws/src/optical_flow_pkg/run_mavros.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 launch mavros mavros.launch.py
EOF
chmod +x ~/rpi_ws/src/optical_flow_pkg/run_mavros.sh

# set_mavros_stream.sh
cat > ~/rpi_ws/src/optical_flow_pkg/set_mavros_stream.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 topic pub /mavros/set_stream mavros_msgs/msg/StreamRate "{stream_id: 0, message_rate: 50, on_off: true}"
EOF
chmod +x ~/rpi_ws/src/optical_flow_pkg/set_mavros_stream.sh

# run_optical_flow.sh
cat > ~/rpi_ws/src/optical_flow_pkg/run_optical_flow.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run optical_flow_pkg optical_flow.py
EOF
chmod +x ~/rpi_ws/src/optical_flow_pkg/run_optical_flow.sh

# run_all_tmux.sh
cat > ~/rpi_ws/src/optical_flow_pkg/run_all_tmux.sh << 'EOF'
#!/bin/bash
SESSION="pilot"
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "TMUX session $SESSION уже существует. Присоединяемся..."
    tmux attach -t $SESSION
    exit 0
fi

tmux new-session -d -s $SESSION
tmux send-keys -t $SESSION "bash ~/rpi_ws/src/optical_flow_pkg/run_camera.sh" C-m
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "bash ~/rpi_ws/src/optical_flow_pkg/run_mavros.sh" C-m
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "bash ~/rpi_ws/src/optical_flow_pkg/run_optical_flow.sh" C-m
tmux attach -t $SESSION
EOF
chmod +x ~/rpi_ws/src/optical_flow_pkg/run_all_tmux.sh

echo "Скрипты запуска созданы."
