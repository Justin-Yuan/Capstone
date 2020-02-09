#ifndef PLANNERS_HEADER
#define PLANNERS_HEADER

// Regular

#include <chrono>
#include <cstdint>
#include <cmath>
#include <eStop.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <random>

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

private:
    teleController eStop;
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber bumper_sub, laser_sub, odom;
    float posX = 0.0, posY = 0.0, yaw = 0.0, currYaw = 0.0;
    uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

    // Debug mode
    bool debug = true;
    bool simulation = true;

    // Speed caps
    double linear_max = 0.20;
    double angular_max = M_PI / 6;

    // Laser variables
    float minLaserDist = std::numeric_limits<float>::infinity();
    float minLeftLaserDist = std::numeric_limits<float>::infinity();
    float minRightLaserDist = std::numeric_limits<float>::infinity();
    int32_t nLasers=0, desiredNLasers=0;
    float laserScanTime = 1.0;
    int laserSize = 0, laserOffset = 0, desiredAngle = 15;
    int right_index = 0, left_index = 0;
    int spin_counter = 0;
    double prevX = 0, prevY = 0;
    float allowed_laser_diff_index = 100;
    float allowed_laser_diff_lr = 0.2;
    float k_p_small = 0.5 * angular_max; // PID controller for angle based on laser difference
    float k_p_big = 1.5 * k_p;


    // Determine mode - and timing stuff
    int mode;
    #define FORWARD 1           // move to the furthest
    #define EXPLORE 2           // explore randomly
    #define time_step 30.       // TODO: remember, time is in seconds!!!!!
    #define time_total 480.
    std::chrono::time_point<std::chrono::system_clock> time_start = std::chrono::system_clock::now();
    uint64_t time_passed = 0;   // initialize the time variable
    uint64_t time_last_update = 0;
    float random_prob = 0.; // the preferrance of exploring randomly increases over time
    bool goRandom;

    // Misc constants
    // double M_PI = 3.1415926535897932384626;
    #define CW false
    #define FRONT true
    #define cos30 cos(M_PI / 6)
    #define turnBack 180

    // Stopping
    float bumperPullbackDist = 0.18;
    float obstacleDist = 0.6;
    float obstacleDist_side = 0.7;
    float obstacleDist_zone = 0.1;

    // Exploration
    int explore_per_dist = 2;
    float exploreDist = 0.5;
    float exploreDist_lr = exploreDist * cos30;
    float exploreDist_side = 1.0;
    int exploreAngle_bins = 12;
    int exploreAngle_size = 360 / exploreAngle_bins;
    // Divide into 4 groups evenly
    vector<double> exploreZone_front = {exploreAngle_bins * 7. / 8., exploreAngle_bins * 1. / 8.}; // > or <
    vector<double> exploreZone_left = {exploreAngle_bins * 1. / 8., exploreAngle_bins * 3. / 8.}; // > and <
    vector<double> exploreZone_back = {exploreAngle_bins * 3. / 8., exploreAngle_bins * 5. / 8.}; // > and <
    vector<double> exploreZone_right = {exploreAngle_bins * 5. / 8., exploreAngle_bins * 7. / 8.}; // > and <
    // // Divide into uneven groups
    // vector<double> exploreZone_front = {exploreAngle_bins * 6. / 8., exploreAngle_bins * 2. / 8.}; // > or <
    // vector<double> exploreZone_left = {exploreAngle_bins * 2. / 8., exploreAngle_bins * 3. / 8.};  // > and <
    // vector<double> exploreZone_back = {exploreAngle_bins * 3. / 8., exploreAngle_bins * 5. / 8.};  // > and <
    // vector<double> exploreZone_right = {exploreAngle_bins * 5. / 8., exploreAngle_bins * 6. / 8.}; // > and <

    // Planning Functions
    void plannerMain();
    void checkBumpers();
    geometry_msgs::Twist threeRegion();

    /* Rotation Functions */
    bool inRange(int bin, const vector<double> & binRange, bool front=false);
    void rotate2angle(float angle, bool CCW=true);
    void rotate2explore(bool CCW=true);
    void rotate2bin(int bin);
    void chooseDirection();

    /* Adjustment Functions */
    float stayAwayFromWalls(float leftDist, float rightDist);
    float stayCentered(float leftDist, float rightDist);
    float stayChill(float frontDist);

    /* Helper Functions */
    void publishVelocity(float angular, float linear, bool spinOnce = false);
    float dist(float startX, float startY, float endX, float endY);

    /* Mode Functions */
    void setMode();

    /* Callback Functions */
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

        // TODO: remember to turn this off for the actual contest
        if (simulation)
        {
            linear_max = 0.20;
        }

        //Initial mode
        mode = FORWARD;
    }

    ~motionPlanner(){ }

    // Functions
    void startup();
    void step();
};

#endif