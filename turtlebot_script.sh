echo "This script will install ROS, Turtlebot packages, and other useful tools."
echo "Press enter to continue, Ctrl-C to cancel"
read

# Install ROS:
echo
echo "Installing ROS packages"
echo
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y


# Setup workstation:
echo
echo "Setting up catkin workspace"
echo
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Turtlebot setup:
echo
echo "Installing turtlebot packages"
echo
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs -y
rosrun kobuki_ftdi create_udev_rules
echo "export TURTLEBOT_3D_SENSOR=kinect" >> ~/.bashrc

# All the additional repositories:
echo
echo "Installing Visual Studio Code and Terminator"
echo
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo add-apt-repository -y ppa:gnome-terminator

sudo apt-get update -y

# Install new packages:
sudo apt-get install code -y
sudo apt-get install terminator -y
sudo apt-get install ros-kinetic-audio-common -y

sudo apt-get upgrade -y

#Finishing 
source ~/.bashrc
