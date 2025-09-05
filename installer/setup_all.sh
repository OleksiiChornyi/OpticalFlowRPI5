#!/bin/bash
chmod +x ./OpticalFlowRPI5/installer/install_ros2_mavros.sh 
./OpticalFlowRPI5/installer/install_ros2_mavros.sh 

chmod +x ./OpticalFlowRPI5/installer/install_camera_deps.sh 
./OpticalFlowRPI5/installer/install_camera_deps.sh 

chmod +x ./OpticalFlowRPI5/installer/create_optical_flow.py.sh 
./OpticalFlowRPI5/installer/create_optical_flow.py.sh 

chmod +x ./OpticalFlowRPI5/installer/create_run_scripts.sh
./OpticalFlowRPI5/installer/create_run_scripts.sh

# Tu run tmux
# ~/autorun/run_all_tmux.sh
