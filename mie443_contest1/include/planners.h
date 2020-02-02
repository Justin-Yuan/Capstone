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
    const float posX, posY, yaw;
    const uint8_t *bumper;
    const float minLaserDist;

public:
    // Initialize the return
    vector<float> output_vels;

    // Constructor
    motionPlanner(float _posX, float _posY, float _yaw, float _laser, uint8_t *_bumper) : posX(_posX), posY(_posY), yaw(_yaw), minLaserDist(_laser), bumper(_bumper)
    {
        // Initialize the control variables
        cout << endl
             << endl
             << "Class created" << endl
             << endl
             << endl;
        // cout << "Class created, bumper type:" << type(bumper[0]) << endl;
        int num_vels = 2;
        output_vels = vector<float>(num_vels, 0.);
    }

    ~motionPlanner(){ }

    // Functions
    geometry_msgs::Twist wallFollower(float minLaserDist);

    vector<float> tutorialPlanner();
    // void tutorialPlanner();
};

#endif