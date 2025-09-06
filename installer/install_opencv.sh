sudo apt update && sudo apt upgrade -y
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libxvidcore-dev libx264-dev
sudo apt-get install -y libatlas-base-dev gfortran python3 python3-dev libeigen3-dev libboost-all-dev libsuitesparse-dev
sudo apt install -y libegl-dev libgl1-mesa-dev libopengl-dev libepoxy-dev python3-pip colcon
python3.12 -m pip install --user wheel
sudo dpkg --remove --force-all python3-catkin-pkg
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro
sudo apt --fix-broken install
sudo apt install -y python3-catkin-pkg-modules python3-rospkg-modules python3-rosdistro-modules
sudo apt install -y libopencv-dev python3-opencv ros-jazzy-cv-bridge python3-numpy
pip install pyserial pymavlink
