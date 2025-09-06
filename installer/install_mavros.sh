# Install mavros
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras 
sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport
sudo apt install -y ros-jazzy-camera-calibration
cd ~/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh