#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <random>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
using namespace std;
using std::vector;

// Debug mode
bool debug = true;
bool simulation = true;

// Global variables
double posX, posY, yaw, currYaw;
double x, y;

// Speed caps
double linear_max = 0.20;
double angular_max = M_PI / 6;

// Bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

// Laser variables
double laserRange = 10;
double laserRange_Left = 10, laserRange_Right = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 15;
int right_ind = 0, left_ind = 0;
int spin_counter = 0;
double x_turn = 0, y_turn = 0;
double x_last = 0, y_last = 0;

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
// Start timing, again, in seconds!!
bool goRandom;
std::random_device device;
std::mt19937 gen(device());

// Misc constants
// double M_PI = 3.1415926535897932384626;
#define CW false
#define FRONT true
#define cos30 cos(M_PI / 6)
int explore_per_dist = 2;
float exploreDist = 0.5;
float exploreDist_lr = exploreDist * cos30; // FIXME: might actually need to be / instead of *
float exploreDist_side = 1.0;
int exploreAngle_bins = 12;
int exploreAngle_size = 360 / exploreAngle_bins;
vector<double> exploreZone_front = {exploreAngle_bins * 7. / 8., exploreAngle_bins * 1. / 8.}; // > or <
vector<double> exploreZone_left = {exploreAngle_bins * 1. / 8., exploreAngle_bins * 3. / 8.}; // > and <
vector<double> exploreZone_back = {exploreAngle_bins * 3. / 8., exploreAngle_bins * 5. / 8.}; // > and <
vector<double> exploreZone_right = {exploreAngle_bins * 5. / 8., exploreAngle_bins * 7. / 8.}; // > and <


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Util functions //////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void publishVelocity(float angular, float linear)
{
    ros::NodeHandle node;
    ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    ros::spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mode decision  //////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void setMode() {
    /**
     * Choose a mode depending on the time passed, all variables are global
     */
    ros::spinOnce();
    time_passed =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - time_start).count();
    random_prob = time_passed / time_total;
 
    std::bernoulli_distribution randomOrNot(random_prob);
    goRandom = randomOrNot(gen);
    if (goRandom) {
        mode = EXPLORE;
    } else {
        mode = FORWARD;
    }
    time_last_update = time_passed;

    ROS_INFO("%f seconds, mode: %d", time_passed, mode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rotation ////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double deg2rad(float angle) {
    return angle * M_PI / 180;
}

inline bool inRange(int bin, const vector<double> & binRange, bool front=false){
    /**
     * Check if the bin is in the desired zone
     */
    if (front){
        return bin > binRange[0] || bin < binRange[1];
    }
    else {
        return bin > binRange[0] && bin < binRange[1];
    }
}

void rotate2angle(float angle, bool CCW=true) {
    /**
     * Rotate the robot to disred angle
     * @param  {degree} float : degree in degrees
     * @param  {CCW} bool : default rotation is CCW == turn left, set to false if need to turn right
     */
    // define velocity
    double angular = angular_max;
    if (!CCW)
        angular = angular * -1;

    // constraints
    ros::spinOnce();
    currYaw = yaw;
    double rad = deg2rad(angle); // TODO: maybe move it to before passing to rotate

    // rotate until desired
    while (abs(yaw - currYaw) < rad)
    {
        publishVelocity(angular, 0.0);
    }
}

void rotate2explore(bool CCW=true) {
    /**
     * Rotate the robot until it is heading to a further wall/object
     * @param  {CCW} bool : default rotation is CCW == turn left, set to false if need to turn right
     */
    double angular = angular_max;
    if (!CCW)
        angular = angular * -1;

    // FIXME: change to the logic of when to stop
    // Currently: stop turning (ready to go forward linearly) if there's something far away enough
    while (laserRange < exploreDist || laserRange_Left < exploreDist_lr || laserRange_Right < exploreDist_lr)
    {
        publishVelocity(angular, 0.0);
    }
}

void rotate2bin(int bin)
{
    /**
     * Command center of which rotate to perform
     * @param  {int} bin : bin index
     */
    if (bin < exploreAngle_bins / 2)
    {
        rotate2angle(bin * exploreAngle_size);
    }
    else
    {
        rotate2angle((exploreAngle_bins - bin) * exploreAngle_size, CW);
    }
}

// Correction function - the robot will rotate 360 degrees first. Then it will rotate to the direction that has the most space.
// The robot should choose direction on its left or right side prior to the front side. Also, the robot will not turn back.
void chooseDirection() {
    /**
     * Rotate and choose direction to explore
     */
    // Define variables to store maximum value
    int maxDist_front = 0, maxDist_front_idx = 0; // default is 0 zo that if things go weird, it just moves forward
    int maxDist_side = 0, maxDist_side_idx = 0;

    // Explore the front/left/right zones (no back zone!)
    for (int bin = 0; bin < exploreAngle_bins; bin++)
    {
        ros::spinOnce();

        // TODO: logic is fine, but might need to change the code appearance
        if (inRange(bin, exploreZone_front, FRONT))
        {
            if (laserRange > maxDist_front)
            {
                maxDist_front = laserRange;
                maxDist_front_idx = bin;
            }
        }
        else if (inRange(bin, exploreZone_left) || inRange(bin, exploreZone_right))
        {
            if (laserRange > maxDist_side)
            {
                maxDist_side = laserRange;
                maxDist_side_idx = bin;
            }
        }
        rotate2angle(exploreAngle_size);
    }

    // Prefers to turn to the side in this mode
    if (maxDist_side > exploreDist_side)
    {
        rotate2bin(maxDist_side_idx);
    }
    else
    {
        rotate2bin(maxDist_front_idx);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensors ////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void bumperCallback(const kobuki_msgs::BumperEvent msg) {
    if (msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if (msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if (msg.bumper == 2)
        bumperRight = !bumperRight;
}

// FIXME: change this function to our own
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // laserSize is 639 increments
    laserSize = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    laserOffset = desiredAngle * M_PI / (180 * msg->angle_increment);

    // Print laser scan info
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize,laserOffset);
    // laserOffset is offset from center in both directions for laserRange

    // Define maximum laser range
    laserRange = 11;
    // Determine if the desired angle is lower than the laser angle
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        // Update front range
        for (int i = laserSize / 2 - laserOffset; i < laserSize / 2 + laserOffset; i++)
        {
            if (laserRange > msg->ranges[i])
                laserRange = msg->ranges[i];
        }
        // Update left/right range by getting the minimum reading in the corresponding zone
        laserRange_Left = 11;
        laserRange_Right = 11;
        for (int i = 0; i < laserOffset; i++)
        {
            if (laserRange_Right > msg->ranges[i])
                laserRange_Right = msg->ranges[i];
            right_ind = i;
            if (laserRange_Left > msg->ranges[laserSize - 1 - i])
                laserRange_Left = msg->ranges[laserSize - 1 - i];
            left_ind = laserSize - 1 - i;
        }
    }
    else
    {
        for (int i = 0; i < laserSize; i++)
        {
            if (laserRange > msg->ranges[i])
                laserRange = msg->ranges[i];
        }
    }
    // Clear Out of Range readings
    if (laserRange == 11)
        laserRange = 0;
    if (laserRange_Left == 11)
        laserRange_Left = 0;
    if (laserRange_Right == 11)
        laserRange_Right = 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("Position:(%f,%f) Orientation: %f Rad or %f degrees.",posX,posY,yaw,yaw*180/M_PI);
}

int main(int argc, char **argv) {
    // TODO: remember to turn this off for the actual contest
    if (simulation)
    {
        linear_max = 0.20;
    }



    mode = FORWARD;

    // Offset Calculation
    int left_ind_offset = left_ind - ((laserSize - 1) / 2);
    int right_ind_offset = ((laserSize - 1) / 2) - right_ind;
    
    // ROS setup
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    teleController eStop;
    ros::Subscriber bumper_sub =
        nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10,
                                             &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);
    ros::Publisher vel_pub =
        nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    // Define speed variables
    double angular = 0.0;
    double linear = 0.0;
    geometry_msgs::Twist vel;

    // Initial Mode
    mode = 2;

    // Distance Counter Setup
    x_last = posX;
    y_last = posY;

    chooseDirection();

    while (ros::ok() && time_passed <= time_total)
    {
        // Mode switch - 120-240s mode 1, else mode 2
        // Mode 1 - goes straight, stop when front range is too low.
        // Mode 2 - corrects the distance when it is going straight. Run correction function after it has passed a certain distance
        // if (time_passed > 120 && time_passed < 240)
        // {
        //     mode = 1;
        // }
        // else
        // {
        //     mode = 2;
        // }

        // Reevaluate the mode every certain durtaion
        if (time_passed - time_last_update >= time_step) setMode();

        if (mode == 2)
        {
            // In mode 2 Print information
            ROS_INFO("LeftRange:%f,RightRange: %f", laserRange_Left,
                     laserRange_Right);
        }
        // Correction counter
        if (sqrt((posX - x_turn) * (posX - x_turn) + (posY - y_turn) * (posY - y_turn)) > explore_per_dist && mode == 2)
        {
            x_turn = posX;
            y_turn = posY;
            chooseDirection();
        }

        // Print Robot Info
        // ROS_INFO("Position:(%f,%f) Orientation: %f degrees. Range: % f, ", posX, posY, yaw * 180 / M_PI, laserRange);
        // ROS_INFO("Range:%f", laserRange);
        // ROS_INFO("LeftIndex:%f,RightIndex: %f",left_ind, right_ind);

        ros::spinOnce();

        // .....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        // ...................................
        // Distance between correction - Newer counter (used for testing purposes in smaller maze)
        //  x_turn = x_turn + abs(posX - x_last);
        //  y_turn = y_turn + abs(posY - y_last);
        //  x_last = posX;
        //  y_last = posY;
        //  if (sqrt((x_turn * x_turn) + (y_turn * y_turn)) > 1.5 && mode == 2)
        //  {
        //      x_turn = 0;
        //      y_turn = 0;
        //      chooseDirection();
        //  }
        // max velocity=0.25, angular velocity=M_PI/6

        // Bumper check
        if (bumperRight || bumperCenter || bumperLeft)
        {
            int bump = 0;
            if (bumperRight)
                bump = 1;
            else if (bumperLeft)
                bump = 2;
            x = posX;
            y = posY;
            // Moving Back
            linear = -0.1;
            angular = 0;
            while (sqrt((posX - x) * (posX - x) + (posY - y) * (posY -
                                                                y)) < 0.15)
            {
               publishVelocity(linear, angular);
            }
            linear = 0;
            if (bump == 1)
                rotate2angle(20);
            else if (bump == 2)
                rotate2angle(20, CW);

            // Moving Forward
            linear = 0.1;
            x = posX;
            y = posY;
            while (sqrt((posX - x) * (posX - x) + (posY - y) * (posY - y)) < 0.15)
            {
                publishVelocity(linear, angular);
            }
            linear = 0;
            // going back to the initial direction
            if (bump == 1)
                rotate2angle(20);
            else if (bump == 2)
                rotate2angle(20, CW);
        }

        // Free Space movement
        // Difference between mode 1 and 2
        if (!bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7)
        {
            if (mode == 1) // Define mode 1
            {
                // Define mode 1 speed
                angular = 0.0;
                linear = linear_max;
                // Correction when it is too close to walls
                if (laserRange_Left < 0.5)
                    angular = -angular_max;
                else if (laserRange_Right < 0.5)
                    angular = angular_max;
            }
            else if (mode == 2) // Define mode 2
            {
                // Define linear speed
                angular = 0.0;
                linear = linear_max;
                // Overwrite the angular speed when the distances from two sides are different more than 0.2
                if (laserRange_Left - laserRange_Right > 0.2)
                    angular = (laserRange_Left - laserRange_Right) /
                              laserRange_Left * (0.5 * angular_max);
                else if (laserRange_Right - laserRange_Left > 0.2)
                    angular = (laserRange_Right - laserRange_Left) /
                              laserRange_Right * (-0.5 * angular_max);
                // Determines where distance values on both sides are located, and if a large discrepancy, turns
                if (left_ind_offset + 100 < right_ind_offset)
                    angular = angular_max;
                else if (right_ind_offset + 100 < left_ind_offset)
                    angular = -angular_max;
                // Overwrite the angular speed when it is too close to wall
                if (laserRange_Left < 0.5)
                    angular = -angular_max;
                else if (laserRange_Right < 0.5)
                    angular = angular_max;
            }
        }
        // When the front sensor reading is too low
        else if (!bumperRight && !bumperCenter && !bumperLeft &&
                 laserRange < 0.5)
        {
            // Determine which side has more space
            if (laserRange_Right < laserRange_Left)
            {
                rotate2explore();
            }
            else
            {
                rotate2explore(CW);
            }
        }
        else
        {
            // Tries to stabilize robot when close to hitting wall
            if (laserRange_Left - laserRange_Right > 0.2)
                angular = (laserRange_Left - laserRange_Right) /
                          laserRange_Left * (0.75 * angular_max);
            else if (laserRange_Right - laserRange_Left > 0.2)
                angular = (laserRange_Right - laserRange_Left) /
                          laserRange_Right * (-0.75 * angular_max);
            else
                angular = 0.0;
            // More gradual deceleration, can be extended for more gradual deceleration, more predictable than just
            // decreasing by constant value until close to wall (previous method)
            if (0.6 < laserRange <= 0.65)
                linear = 0.65 * linear_max;
            else if (0.55 < laserRange <= 0.6)
                linear = 0.5 * linear_max;
            else if (0.5 <= laserRange <= 0.55)
                linear = 0.3 * linear_max;
            else
                linear = 0.8 * linear_max;
        }

        // Mode 0 for sensor test
        if (mode == 0)
        {
            angular = 0;
            linear = 0;
        }

        // write the defined speed to the robot
        publishVelocity(angular, linear);
        // The last thing to do is to update the timer.
        time_passed =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - time_start).count();
    }
    return 0;
}