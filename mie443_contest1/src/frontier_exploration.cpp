/* Building map by frontier exploration 
references
- http://cs.unh.edu/~tg1034/project/TianyiGu_AutonomousMapping.pdf
- https://arxiv.org/pdf/1806.03581.pdf
- https://github.com/paulbovbel/frontier_exploration
*/
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

using namespace std; 



