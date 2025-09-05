#!/bin/bash
set -e
echo "===================="
echo "3. Установка зависимостей камеры"
echo "===================="

sudo apt install -y v4l-utils qv4l2 xvfb git python3-pip libboost-dev openssl \
    libgnutls28-dev libtiff-dev pybind11-dev meson cmake libglib2.0-dev libgstreamer-plugins-base1.0-dev

mkdir -p ~/rpi_camera/src
cd ~/rpi_camera/src

if [ ! -d "camera_ros" ]; then
    git clone https://github.com/christianrauch/camera_ros.git
else
    echo "camera_ros уже клонирован, пропускаем"
fi

cd ~/rpi_camera
source /opt/ros/jazzy/setup.bash

echo "Обновляем rosdep..."
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera || true

echo "Сборка camera_ros..."
colcon build --event-handlers=console_direct+ || echo "Build пропущен из-за ошибок (можно пересобрать вручную)"

if ! grep -q "~/rpi_camera/install/setup.bash" ~/.bashrc; then
    echo "source ~/rpi_camera/install/setup.bash" >> ~/.bashrc
fi

echo "Камера ROS2 установлена!"
