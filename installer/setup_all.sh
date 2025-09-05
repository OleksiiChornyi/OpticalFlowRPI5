#!/bin/bash
chmod +x ./OpticalFlowRPI5/installer/install_ros2_mavros.sh 
./install_ros2_mavros.sh

chmod +x ./OpticalFlowRPI5/installer/install_camera_deps.sh 
./install_camera_deps.sh

chmod +x ./OpticalFlowRPI5/installer/create_optical_flow.py.sh 
./create_optical_flow.py.sh

chmod +x ./OpticalFlowRPI5/installer/create_run_scripts.sh
./create_run_scripts.sh

# Tu run tmux
# ~/autorun/run_all_tmux.sh
