#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
using namespace std;

//Define Global Variables
double posX, posY, yaw, yaw_now;
double x, y;
double pi = 3.1416;

//Define Maximum Speed
double linear_max = 0.15;
double angular_max = pi / 6;
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//Define Laser Ranges
double laserRange = 10;
double laserRange_Left = 10, laserRange_Right = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 15;
int right_ind = 0, left_ind = 0;
int spin_counter = 0;
double x_turn = 0, y_turn = 0;
double x_last = 0, y_last = 0;

//Rotation function - with given degree and direction.
//When the degree input is 360, the robot will rotate until all the three laser ranges are greater than 0.5 
void rotate(float degree, char direction)
{
    //Define rad
    double rad;
    rad = degree * pi / 180;
    //Update Odometry
    ros::spinOnce();
    //Record the status
    yaw_now = yaw;
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    //Define the magnitude of linear and angular speed here
    double angular = angular_max;
    double linear = 0.0;
    //Define the rotation direction according to the input
    if (direction == 'r')
    {
        angular = angular * -1;
    }
    //Define 360 degree rotation
    if (degree == 360)
    {
        while (laserRange < 0.7 || laserRange_Left < 0.5 ||
               laserRange_Right < 0.5)
        {
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::spinOnce();
        }
    }
    else //Rotate to certain degree
    {
        while (abs(yaw - yaw_now) < rad)
        {
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::spinOnce();
        }
    }
}
//Correction function - the robot will rotate 360 degrees first. Then it will rotate to the direction that has the most space.
//The robot should choose direction on its left or right side prior to the front side.Also, the robot will not turn back.
void correction()
{
    //Define the number of increments in the 360-degree rotation
    int directions = 24;
    //Define variables to store maximum value
    int max_ind_forward = 0;
    int max_ind_side = 0;
    double max_reading_forward = 0;
    double max_reading_side = 0;
    int max_index = 0;

    //Get the max index
    for (int i = 0; i < directions; i++)
    {
        //Update status
        ros::spinOnce();
        //Edit the size of deadzone
        //Size of forward zone
        if (i < directions / 8 || i > directions * 7 / 8)
        {
            if (laserRange > max_reading_forward)
            {
                max_reading_forward = laserRange;
                max_ind_forward = i;
            }
        }
        //Size of the left/right zone
        else if ((i >= directions / 8 && i <= directions * 3 / 8) || (i >= directions * 5 / 8 && i <= directions * 7 / 8))
        {
            if (laserRange > max_reading_side)
            {
                max_reading_side = laserRange;
                max_ind_side = i;
            }
        }
        rotate(360 / directions, 'r');
    }
    //Determine if it can go left/right. If not, go to the direction in forward zone
    if (max_reading_side > 1.0)
    {
        max_index = max_ind_side;
    }
    else
    {
        max_index = max_ind_forward;
    }
    //Determine the direction and rotate
    if (max_index < directions / 2)
    {
        rotate(360 * max_index / directions, 'r');
    }
    else
    {
        rotate(360 * (directions - max_index) / directions, 'l');
    }
}
//Bumper call back function
void bumperCallback(const kobuki_msgs::BumperEvent msg)
{
    if (msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if (msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if (msg.bumper == 2)
        bumperRight = !bumperRight;
}
//Laser callback function
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //laserSize is 639 increments
    laserSize = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    laserOffset = desiredAngle * pi / (180 * msg->angle_increment);

    //Print laser scan info
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize,laserOffset);
    //laserOffset is offset from center in both directions for laserRange

    //Define maximum laser range
    laserRange = 11;
    //Determine if the desired angle is lower than the laser angle
    if (desiredAngle * pi / 180 < msg->angle_max && -desiredAngle * pi / 180 > msg->angle_min)
    {
        //Update front range
        for (int i = laserSize / 2 - laserOffset; i < laserSize / 2 + laserOffset; i++)
        {
            if (laserRange > msg->ranges[i])
                laserRange = msg->ranges[i];
        }
        //Update left/right range by getting the minimum reading in the corresponding zone
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
    //Clear Out of Range readings
    if (laserRange == 11)
        laserRange = 0;
    if (laserRange_Left == 11)
        laserRange_Left = 0;
    if (laserRange_Right == 11)
        laserRange_Right = 0;
}

//odometry call back function
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //Read Info from odom
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position:(%f,%f) Orientation: %f Rad or %f degrees.",posX,posY,yaw,yaw*180/pi);
}

int main(int argc, char **argv)
{
    //Set Initial Mode
    int mode = 1;
    //Offset Calculation
    int left_ind_offset = left_ind - ((laserSize - 1) / 2);
    int right_ind_offset = ((laserSize - 1) / 2) - right_ind;
    //ROS setup
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

    //Define speed variables
    double angular = 0.0;
    double linear = 0.0;
    geometry_msgs::Twist vel;

    //Set timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0;              // the timer just started, so we know it is less than 480, no need to check.int time_last = 0;

    //Initial Mode
    mode = 2;

    //Distance Counter Setup
    x_last = posX;
    y_last = posY;

    correction();

    while (ros::ok() && secondsElapsed <= 480)
    {
        //Mode switch - 120-240s mode 1, else mode 2
        //Mode 1 - goes straight, stop when front range is too low.
        //Mode 2 - corrects the distance when it is going straight. Run correction function after it has passed a certain distance
        if (secondsElapsed > 120 && secondsElapsed < 240)
        {
            mode = 1;
        }
        else
        {
            mode = 2;
        }
        if (mode == 2)
        {
            //In mode 2 Print information
            ROS_INFO("LeftRange:%f,RightRange: %f", laserRange_Left,
                     laserRange_Right);
        }
        //Correction counter
        if (sqrt((posX - x_turn) * (posX - x_turn) + (posY - y_turn) * (posY - y_turn)) > 1.5 && mode == 2)
        {
            x_turn = posX;
            y_turn = posY;
            correction();
        }

        //Print Robot Info
        //ROS_INFO("Position:(%f,%f) Orientation: %f degrees. Range: % f, ", posX, posY, yaw * 180 / pi, laserRange);
        //ROS_INFO("Range:%f", laserRange);
        //ROS_INFO("LeftIndex:%f,RightIndex: %f",left_ind, right_ind);

        ros::spinOnce();

        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................
        //Distance between correction - Newer counter (used for testing purposes in smaller maze)
        // x_turn = x_turn + abs(posX - x_last);
        // y_turn = y_turn + abs(posY - y_last);
        // x_last = posX;
        // y_last = posY;
        // if (sqrt((x_turn * x_turn) + (y_turn * y_turn)) > 1.5 && mode == 2)
        // {
        //     x_turn = 0;
        //     y_turn = 0;
        //     correction();
        // }
        //max velocity=0.25, angular velocity=pi/6

        //Bumper check
        if (bumperRight || bumperCenter || bumperLeft)
        {
            int bump = 0;
            if (bumperRight)
                bump = 1;
            else if (bumperLeft)
                bump = 2;
            x = posX;
            y = posY;
            //Moving Back
            linear = -0.1;
            angular = 0;
            while (sqrt((posX - x) * (posX - x) + (posY - y) * (posY -
                                                                y)) < 0.15)
            {
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
            }
            linear = 0;
            if (bump == 1)
                rotate(20, 'l');
            else if (bump == 2)
                rotate(20, 'r');

            //Moving Forward
            linear = 0.1;
            x = posX;
            y = posY;
            while (sqrt((posX - x) * (posX - x) + (posY - y) * (posY - y)) < 0.15)
            {
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
            }
            linear = 0;
            //going back to the initial direction
            if (bump == 1)
                rotate(20, 'r');
            else if (bump == 2)
                rotate(20, 'l');
        }

        //Free Space movement
        //Difference between mode 1 and 2
        if (!bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7)
        {
            if (mode == 1) //Define mode 1
            {
                //Define mode 1 speed
                angular = 0.0;
                linear = linear_max;
                //Correction when it is too close to walls
                if (laserRange_Left < 0.5)
                    angular = -angular_max;
                else if (laserRange_Right < 0.5)
                    angular = angular_max;
            }
            else if (mode == 2) //Define mode 2
            {
                //Define linear speed
                angular = 0.0;
                linear = linear_max;
                //Overwrite the angular speed when the distances from two sides are different more than 0.2
                if (laserRange_Left - laserRange_Right > 0.2)
                    angular = (laserRange_Left - laserRange_Right) /
                              laserRange_Left * (0.5 * angular_max);
                else if (laserRange_Right - laserRange_Left > 0.2)
                    angular = (laserRange_Right - laserRange_Left) /
                              laserRange_Right * (-0.5 * angular_max);
                //Determines where distance values on both sides are located, and if a large discrepancy, turns
                if (left_ind_offset + 100 < right_ind_offset)
                    angular = angular_max;
                else if (right_ind_offset + 100 < left_ind_offset)
                    angular = -angular_max;
                //Overwrite the angular speed when it is too close to wall
                if (laserRange_Left < 0.5)
                    angular = -angular_max;
                else if (laserRange_Right < 0.5)
                    angular = angular_max;
            }
        }
        //When the front sensor reading is too low
        else if (!bumperRight && !bumperCenter && !bumperLeft &&
                 laserRange < 0.5)
        {
            //Determine which side has more space
            if (laserRange_Right > laserRange_Left)
            {
                rotate(360, 'r');
            }
            else
            {
                rotate(360, 'l');
            }
        }
        else
        {
            //Tries to stabilize robot when close to hitting wall
            if (laserRange_Left - laserRange_Right > 0.2)
                angular = (laserRange_Left - laserRange_Right) /
                          laserRange_Left * (0.75 * angular_max);
            else if (laserRange_Right - laserRange_Left > 0.2)
                angular = (laserRange_Right - laserRange_Left) /
                          laserRange_Right * (-0.75 * angular_max);
            else
                angular = 0.0;
            //More gradual deceleration, can be extended for more gradual deceleration, more predictable than just
            //decreasing by constant value until close to wall (previous method)
            if (0.6 < laserRange <= 0.65)
                linear = 0.65 * linear_max;
            else if (0.55 < laserRange <= 0.6)
                linear = 0.5 * linear_max;
            else if (0.5 <= laserRange <= 0.55)
                linear = 0.3 * linear_max;
            else
                linear = 0.8 * linear_max;
        }

        //Mode 0 for sensor test
        if (mode == 0)
        {
            angular = 0;
            linear = 0;
        }

        //write the defined speed to the robot
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        //The last thing to do is to update the timer.
        secondsElapsed =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    }
    return 0;
}