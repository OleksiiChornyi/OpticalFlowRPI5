#!/bin/bash
set -e
echo "=== Установка MAVROS ==="

sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo apt install -y ros-humble-rqt ros-humble-rqt-image-view ros-humble-compressed-image-transport
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
