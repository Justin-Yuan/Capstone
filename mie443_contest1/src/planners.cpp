// #include "mie443_contest1/include/planners.h"
#include "planners.h"

geometry_msgs::Twist motionPlanner::wallFollower(float minLaserDist, float dt) 
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
    output.angular.z = controlYaw;
    output.linear.x = forwardSpeed;


    prevYaw = controlYaw;
    prevError = currError;
    ROS_INFO("Error: (%f), Error Derivative: (%f), Control Yaw: %f, Forward Speed: %f, Time Step: %f", 
        currError, currErrorDeriv, controlYaw, forwardSpeed, dt);
    return output;
}

geometry_msgs::Twist motionPlanner::threeRegion(float minLaserDist) 
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
        controlYaw = 4;
        forwardSpeed = 0;
    } 
    else if (currentDistFromWall > 2) 
    {
        controlYaw = -1;
    }
    output.angular.z = controlYaw;
    output.linear.x = forwardSpeed;
    return output;
}
