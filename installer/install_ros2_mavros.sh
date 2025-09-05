#!/bin/bash
set -e
echo "===================="
echo "1. Установка ROS2 и MAVROS"
echo "===================="

# Обновление системы
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales curl gnupg lsb-release software-properties-common

# Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Добавляем ключ ROS2 и репозиторий
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Установка ROS2 base
sudo apt install -y ros-jazzy-ros-base python3-rosdep python3-colcon-common-extensions python3-argcomplete

# Инициализация rosdep (если ещё не выполнена)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Установка MAVROS
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport ros-jazzy-camera-calibration

# Установка GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O ~/install_geographiclib_datasets.sh
chmod +x ~/install_geographiclib_datasets.sh
sudo ~/install_geographiclib_datasets.sh

echo "ROS2 и MAVROS установлены!"
