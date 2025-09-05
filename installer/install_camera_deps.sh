#!/bin/bash
set -e
echo "===================="
echo "2. Установка зависимостей камеры"
echo "===================="

# USB / V4L2 камера
sudo apt install -y v4l-utils qv4l2 xvfb git python3-pip libboost-dev openssl \
    libgnutls28-dev libtiff-dev pybind11-dev meson cmake libglib2.0-dev libgstreamer-plugins-base1.0-dev

# Создание рабочей папки для ROS2 camera
mkdir -p ~/rpi_camera/src
cd ~/rpi_camera/src

# Клонируем camera_ros
git clone https://github.com/christianrauch/camera_ros.git
cd ~/rpi_camera

# Установка зависимостей ROS2
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera

# Сборка пакета
colcon build --event-handlers=console_direct+
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
echo "source ~/rpi_camera/install/setup.bash" >> ~/.bashrc
sudo chmod -R 777 /dev/

echo "Камера ROS2 установлена!"
