#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

// Which bumper is hit -> which direction should we go
// Updated in the bumperCallback and used in main
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0; 

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    ROS_INFO("BUUUUUMP!!!");
    ROS_INFO("Bumper: %d", msg->bumper);
    ROS_INFO("Bumper State: %d", msg->state);
    if (msg->bumper == 0)
        bumperLeft = !bumperLeft;
    else if (msg->bumper == 1)
        bumperCenter = !bumperCenter;
    else if (msg->bumper == 2)
        bumperRight = !bumperRight;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
