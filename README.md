# Capstone
UofT MIE443 Capstone 

system requirements: Ubuntu 16.04 

run setup.sh to install ROS and Turtlebot2 (might take a while)

all packages need to be placed under ~/catkin_ws/src, could be done be creating soft links 

\*bin/, \*build and \*devel directories are ignored from git 

in order for os to locate our `mie443_contest1` package, do this 
```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Capstone
```

