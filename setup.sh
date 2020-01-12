#!/bin/bash

HOME=`pwd`

# update apt 
sudo apt update
sudo apt upgrade

# install useful tools 
sudo apt-get install tmux 
sudo apt-get install curl 
sudo apt-get install git 

# install ROS 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update 

sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# make ROS workspace 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH

# # install Turtlebot2 (extra dependencies ??? may not be needed)
# sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard 
# sudo apt-get install ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan 
# sudo apt-get install ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server 
# sudo apt-get install ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server 
# sudo apt-get install ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport 
# sudo apt-get install ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

# kinetic Turtlebot2
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator 
sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs ros-kinetic-turtlebot-gazebo
sudo apt-get install ros-kinetic-rocon-remocon ros-kinetic-rocon-qt-library
# if ros-kinetic-rocon-remocon ros-kinetic-rocon-qt-library not found, manually install them 
# reference: http://mario.is-programmer.com/posts/212006.html
cd ~/catkin_ws/src
git clone https://github.com/robotics-in-concert/rocon_qt_gui.git
sudo apt-get install pyqt4-dev-tools
sudo apt-get install pyqt5-dev-tools
cd ~/catkin_ws/
catkin_make

# soft links for git 
ln -s $HOME/mie443_contest1 ~/catkin_ws/src/mie443_contest1
ln -s $HOME/mie443_contest2 ~/catkin_ws/src/mie443_contest2
ln -s $HOME/mie443_contest3/mie443_contest3 ~/catkin_ws/src/mie443_contest3
ln -s $HOME/mie443_contest3/turtlebot_follower ~/catkin_ws/src/turtlebot_follower


# # test launch 
# source /opt/ros/kinetic/setup.bash 
# # # otherwise do 
# # export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world
# roslaunch turtlebot_gazebo turtlebot_world.launch

# to build 
#### Running command: "make -j4 -l4" in "/home/justiny/catkin_ws/build"
