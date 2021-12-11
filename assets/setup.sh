#!/bin/sh

set -e

# Record the time this script starts
date

# Get the full dir name of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Keep updating the existing sudo time stamp
sudo -v
while true; do sudo -n true; sleep 120; kill -0 "$$" || exit; done 2>/dev/null &

#echo "\e[100m Remove OpenCV 4 \e[0m"
#sudo find /usr/ -name "*opencv*4*" -delete

echo "\e[100m Build Jetson Inference \e[0m"
cd ~
sudo apt-get -y install cmake
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~

echo "\e[100m Install ROS Eloquent Elusor \e[0m"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-eloquent-desktop \
                    python3-colcon-common-extensions \
                    python3-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
#source /opt/ros/eloquent/setup.bash

echo "\e[100m Download ROS2 Packages \e[0m"
echo "\e[100m 1. image_pipeline \e[0m"
sudo apt-get install -y ros-eloquent-camera-info-manager \
                        ros-eloquent-launch-testing-ament-cmake \
                        python3-opencv
cd ~/ros_workspace/src
git clone https://github.com/harsha-vk/image_pipeline.git
cd image_pipeline
git checkout dashing
rm -rf image_publisher

echo "\e[100m 2. ros_deep_learning \e[0m"
sudo apt-get install -y ros-eloquent-vision-msgs \
                        ros-eloquent-launch-xml \
                        ros-eloquent-launch-yaml
cd ~/ros_workspace/src
git clone https://github.com/harsha-vk/ros_deep_learning.git
git checkout ros2_1

echo "\e[100m 3. ublox \e[0m"
sudo apt-get install libasio-dev \
                     ros-eloquent-diagnostic-updater
cd ~/ros_workspace/src
git clone https://github.com/KumarRobotics/ublox.git
cd ublox
git checkout dashing-devel

echo "\e[100m 4. imu_tools \e[0m"
cd ~/ros_workspace/src
git clone https://github.com/ccny-ros-pkg/imu_tools.git
cd imu_tools
git checkout eloquent

echo "\e[100m 5. robot_localization \e[0m"
cd ~/ros_workspace/src
git clone https://github.com/cra-ros-pkg/robot_localization.git
cd robot_localization
git checkout eloquent-devel