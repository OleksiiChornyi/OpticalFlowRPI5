#!/bin/bash
chmod +x install_ros2_mavros.sh install_camera_deps.sh create_optical_flow.py.sh create_run_scripts.sh

./install_ros2_mavros.sh
./install_camera_deps.sh
./create_optical_flow.py.sh
./create_run_scripts.sh

# Tu run tmux
# ~/autorun/run_all_tmux.sh
