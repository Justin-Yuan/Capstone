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

### Kill ros processes (forcefully)
```bash
pkill -f ros
```

### Reset Gazebo (properly)
http://answers.gazebosim.org/question/15085/gazebo-world-reset-using-ros/


## Launch 

### Version 1
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

### Version 2
Note: First, remember to setup workspace before running any contest package/code (only once)
```bash
source catkin_ws/devel/setup.bash 
```

run simulation with 1 command 
- execute `roscore` in separate terminal only once 
- every time you want to run contest 1, go to **~/catkin_ws/src/Capstone/mie443_contest1** and launch 
```bash
bash run.sh 
``` 
- by default, it loads world/robot, launches gmapping/slam, rviz and runs the control executable 
- use `-e <name>` or `--executable <name>` to run different executable, e.g. `contest1` or `reference`
- to kill after finish running, press **Enter** in the launch 
terminal (this will kill all child terminals)
- you can save map by enabling `-s` or `--save_map`, this will save the map named **map** after shutdown; to change map name, provide argument `-m <name>` or `--map_name <name>`


## Miscellaneous

to reset gazebo simulation without shutting down (don't seem to work tho)
```bash
rosservice call /gazebo/reset_world
```

## Links
MIE quercus page: https://q.utoronto.ca/courses/139490


## IP address 
get ip address 
```bash
hostname -I | awk ’{print $1}’
```
S09 machine: 
100.64.76.236
9 machine:

launch vnc on turtlebot machine 
```bash
x11vnc -ncache 10
```
port is 5900


## TODO LIST
1. Basic Algorithm for the robot to run in simulation. E.G. Wall Following
2. Reference code. Transfer util functions to what we need: 
    rotate
    correction
    laser callback
    odometry callback
    mode1 / mode2
    general exploration algrithm
3. Object detection :
    get the image from gmapping 
    process the image to find the objects (bins)
    find a way to label two bins
4. Try manipulate Occupancy Grid Map 
5. Curve Detector for labelling the bins
6. Report
