# Capstone
UofT MIE443 Capstone 


## Setup 
system requirements: Ubuntu 16.04 

run turtlebot_script.sh to install ROS and Turtlebot2 (might take a while)

\*bin/, \*build and \*devel directories are ignored from git 

in order for os to locate our `mie443_contest1` package, do this in bashrc file 
```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Capstone
```

remember to setup workspace before running any contest package/code 
```bash
source catkin_ws/devel/setup.bash 
```


## Launch 

simulation run for contest 1, for each of the following line, launsh in a separate terminal 
```bash
roscore
# load map and bot 
roslaunch mie443_contest1 turtlebot_world.launch
# run slam 
roslaunch mie443_contest1 gmapping.launch
# run rviz viewer
roslaunch turtlebot_rviz_launchers view_navigation.launch   
# control logic 
rosrun mie443_contest1 contest1

# optional, view subscribed messages (e.g. for bumper)
rostopic echo /mobile_bash/events/bumper​
```


## Miscellaneous

to reset gazebo simulation without shutting down (don't seem to work tho)
```bash
rosservice call /gazebo/reset_world
```

## Links
MIE quercus page: https://q.utoronto.ca/courses/139490
