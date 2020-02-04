// #include "mie443_contest1/include/planners.h"
#include "planners.h"

void motionPlanner::step()
{
    ros::spinOnce();
    checkBumpers();
    //vel = planner.wallFollower(loop_rate.expectedCycleTime().toSec());
    threeRegion();
}


/* Private Functions */

void motionPlanner::checkBumpers()
{
    if (bumper[kobuki_msgs::BumperEvent::LEFT] == 1 || 
        bumper[kobuki_msgs::BumperEvent::CENTER] == 1 || 
        bumper[kobuki_msgs::BumperEvent::RIGHT] == 1)
    {
        float startX, startY;

        // Maneuver backwards
        startX = posX; startY = posY;
        while (dist(startX, startY, posX, posY) < 0.15)
        {
           publishVelocity(-0.1 /* linear */, 0 /* angular */, true /* spinOnce */);
        }

        // Adjust angle away from obstacle
        // if (bumper[kobuki_msgs::BumperEvent::RIGHT] == 1)
        //     // rotate2angle(20);
        // else if (bumper[kobuki_msgs::BumperEvent::LEFT] == 1)
            // rotate2angle(20, CW);

        // Maneuver forwards
        startX = posX; startY = posY;
        while (dist(startX, startY, posX, posY) < 0.15)
        {
           publishVelocity(0.1 /* linear */, 0 /* angular */, true /* spinOnce */);
        }

        // // Adjust angle back to original direction
        // if (bumper[kobuki_msgs::BumperEvent::RIGHT] == 1)
        //     // rotate2angle(20);
        // else if (bumper[kobuki_msgs::BumperEvent::LEFT] == 1)
            // rotate2angle(20, CW);
    }
}

geometry_msgs::Twist motionPlanner::wallFollower(float dt) 
{
    /**
     * Controls the robot to follow the wall. Uses three regions control.
     * @param  {minLaserDist} float : the minimum value in msg->ranges from our laser scan - distance to closest wall.
     * @param  {dt} float : the time between planner calls. Calculated from the loop_rate set in contest1.cpp
     */
    geometry_msgs::Twist output;

    float desiredDistFromWall = 1;
    float currentDistFromWall = minLaserDist;
    float kp = 2.0;
    float kd = 1.0;

    // Control yaw using PD controller. We set forward speed to a constant.
    float currError = desiredDistFromWall - currentDistFromWall;
    float currErrorDeriv = (currError - prevError) / dt;

    float pGain = kp * currError;
    float dGain = kd * currErrorDeriv;
    float controlYaw = pGain + dGain;
    float forwardSpeed = 0.2;

    // If our laser scan input is too large, we maintain the last reasonable yaw input.
    if ((controlYaw > 5)||(controlYaw < -5)) 
    {
        controlYaw = prevYaw;
    }
    publishVelocity(controlYaw, forwardSpeed);

    prevYaw = controlYaw;
    prevError = currError;
    ROS_INFO("Error: (%f), Error Derivative: (%f), Control Yaw: %f, Forward Speed: %f, Time Step: %f", 
        currError, currErrorDeriv, controlYaw, forwardSpeed, dt);
}

geometry_msgs::Twist motionPlanner::threeRegion() 
{
    /**
     * Controls the robot to follow the wall. Uses three regions control.
     * @param  {minLaserDist} float : the minimum value in msg->ranges from our laser scan - distance to closest wall.
     */
    geometry_msgs::Twist output;

    float closeThreshold = 0.5;
    float currentDistFromWall = minLaserDist;
    float controlYaw = 0;
    float forwardSpeed = 0.1;

    // If we turn outwards if we are too close to wall and we turn inwards if we are too far from wall.
    if (currentDistFromWall < closeThreshold) 
    {
        controlYaw = 2;
        forwardSpeed = 0;
    } 
    else if (currentDistFromWall > 2) 
    {
        controlYaw = -1;
    }
    publishVelocity(controlYaw, forwardSpeed);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions ////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

float motionPlanner::dist(float startX, float startY, float endX, float endY)
{
    return sqrt(pow(endX - startX, 2) + pow(endY - startY, 2));
}

void motionPlanner::publishVelocity(float angular, float linear, bool spinOnce /*= false*/)
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    if (spinOnce) 
    {
        ros::spinOnce();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback functions //////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motionPlanner::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void motionPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    laserScanTime = msg->scan_time;
    ROS_INFO("Size of laser scan array: %i, size of offset: %i, angle_max: %f, angle_min: %f, range_max: %f, range_min: %f, angle_increment: %f, scan_time: %f", 
        nLasers, desiredNLasers, msg->angle_max, msg->angle_min, msg->range_max, msg->range_min, msg->angle_increment, msg->scan_time);

    minLaserDist = msg->range_max;
    minLeftLaserDist = msg->range_max;
    minRightLaserDist = msg->range_max;
    for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
    {
        if (msg->range_max > msg->ranges[laser_idx] > 0) 
        {
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    for (uint32_t laser_idx = 0; laser_idx < nLasers/2; ++laser_idx)
    {
        if (msg->range_max > msg->ranges[laser_idx] > 0) 
        {
                minRightLaserDist = std::min(minRightLaserDist, msg->ranges[laser_idx]);
        }
        if (msg->range_max > msg->ranges[nLasers-laser_idx-1] > 0) 
        {
                minLeftLaserDist = std::min(minLeftLaserDist, msg->ranges[nLasers-laser_idx-1]);
        }
    }
    ROS_INFO("MinLaserDist: %f, minLeftLaserDist: %f, minRightLaserDist: %f", minLaserDist, minLeftLaserDist, minRightLaserDist);
}

void motionPlanner::odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}
