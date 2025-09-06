# Install camera images reader
# TODO: Works manually, but camera install fails when running automated somewhere...
sudo apt update
sudo apt install -y v4l-utils qv4l2 # to USB camera
sudo apt install xvfb # to fake camera if needed
sudo apt install -y git python3-pip python3-jinja2 libboost-dev openssl
sudo apt install -y libgnutls28-dev libtiff-dev pybind11-dev meson cmake
sudo apt install -y python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev

git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
sudo ninja -C build install

mkdir -p ~/rpi_camera/src
cd ~/rpi_camera/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/rpi_camera
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera

colcon build --event-handlers=console_direct+
echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/libcamera.conf
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo "sudo chmod -R 777 /dev/" >> ~/.bashrc
echo "source ~/rpi_camera/install/setup.bash" >> ~/.bashrc
sudo chmod -R 777 /dev/
source ~/rpi_camera/install/setup.bash