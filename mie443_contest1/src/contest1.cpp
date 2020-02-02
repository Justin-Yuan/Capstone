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
// #include "mie443_contest1/include/planners.h"
#include "planners.h"

using namespace std;


#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// global vars
float posX = 0.0, posY = 0.0, yaw = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    //ROS_INFO("Size of laser scan array: %i, size of offset: %i, angle_max: %f, angle_min: %f, range_max: %f, range_min: %f, angle_increment: %f", 
        //nLasers, desiredNLasers, msg->angle_max, msg->angle_min, msg->range_max, msg->range_min, msg->angle_increment);

    minLaserDist = 10;
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if (msg->range_max > msg->ranges[laser_idx] > 0) {
                    minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }
        }
    } else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            if (msg->range_max > msg->ranges[laser_idx] > 0) {
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }
        }
    }

    if (minLaserDist == 10) {
        ROS_INFO("Robot stuck in corner!");
        minLaserDist = 2;
    }
    ROS_INFO("MinLaserDist: %f", minLaserDist);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    motionPlanner planner(posX, posY, yaw, minLaserDist, bumper);
    while(ros::ok() && secondsElapsed <= 480) {
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees, MinLaserDist: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        ros::spinOnce();

        // Obtain movement command from planner
        vel = planner.wallFollower(minLaserDist);
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
