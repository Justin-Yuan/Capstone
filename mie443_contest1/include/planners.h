#ifndef PLANNERS_HEADER
#define PLANNERS_HEADER

// Regular
#include <cstdint>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <iostream>

// ROS
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;
using std::vector;

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

class motionPlanner {
// If we really want, we can have another "stupidPlanner" class inherenting from this one
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber bumper_sub, laser_sub, odom;
    float posX = 0.0, posY = 0.0, yaw = 0.0;
    uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

    float minLaserDist = std::numeric_limits<float>::infinity();
    float minLeftLaserDist = std::numeric_limits<float>::infinity();
    float minRightLaserDist = std::numeric_limits<float>::infinity();
    int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;
    float laserScanTime = 1.0;

    float prevYaw = 1.0;
    float prevError = 0.0;

    // Functions
    void checkBumpers();

    geometry_msgs::Twist wallFollower(float dt);
    geometry_msgs::Twist threeRegion();

    void publishVelocity(float angular, float linear, bool spinOnce = false);
    float dist(float startX, float startY, float endX, float endY);

    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);

public:
    // Initialize the return
    vector<float> output_vels;

    // Constructor
    motionPlanner()
    {
        // Initialize the control variables
        cout << endl
             << endl
             << "Class created" << endl
             << endl
             << endl;

        bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &motionPlanner::bumperCallback, this);
        laser_sub = nh.subscribe("scan", 10, &motionPlanner::laserCallback, this);
        odom = nh.subscribe("odom", 1, &motionPlanner::odomCallback, this);

        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    }

    ~motionPlanner(){ }

    // Functions
    void step();
};

#endif