#!/bin/bash
set -e
echo "===================="
echo "2. Установка MAVROS"
echo "===================="

sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport
sudo apt install -y ros-jazzy-camera-calibration

# GeographicLib datasets
if [ ! -f ~/install_geographiclib_datasets.sh ]; then
    wget -O ~/install_geographiclib_datasets.sh https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod a+x ~/install_geographiclib_datasets.sh
fi
sudo ~/install_geographiclib_datasets.sh

echo "MAVROS установлен!"
