// #include "mie443_contest1/include/planners.h"
#include "planners.h"

geometry_msgs::Twist motionPlanner::wallFollower(float minLaserDist, float dt) 
{
    geometry_msgs::Twist output;

    float desiredDistFromWall = 1;
    float currentDistFromWall = minLaserDist;
    float kp = 2.0;
    float kd = 1.0;

    // Get yaw using PD controller. We set forward speed to a constant.
    float currError = desiredDistFromWall - currentDistFromWall;
    float currErrorDeriv = (currError - prevError) / dt;

    float pGain = kp * currError;
    float dGain = kd * currErrorDeriv;
    float controlYaw = pGain + dGain;
    float forwardSpeed = 0.2;

    // If our laser scan input is too large, we maintain the last reasonable yaw input.
    if ((controlYaw > 5)||(controlYaw < -5)) {
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

vector<float> motionPlanner::tutorialPlanner()
{
    cout << "TutorialPlanner in use..." << endl;
    float angular = 0.0;
    float linear = 0.0;

    // Check if any of the bumpers were pressed.
    bool any_bumper_pressed = false;
    bool pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        pressed = (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        any_bumper_pressed |= pressed;
        ROS_INFO("Bumper: (%i) Pressed: %s", b_idx, pressed?"true":"false");
    }
    // Control logic after bumpers are being pressed.
    //
    ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
    if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
        angular = 0.0;
        linear = 0.2;
    }
    else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
        angular = M_PI / 6;
        linear = 0.0;
    }
    else if (minLaserDist > 1. && !any_bumper_pressed) {
        linear = 0.1;
        if (yaw < 17 / 36 * M_PI || posX > 0.6) {
            angular = M_PI / 12.;
        }
        else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
            angular = -M_PI / 12.;
        }
        else {
            angular = 0;
        }
    }
    else {
        angular = 0.0;
        linear = 0.0;
    }

    output_vels[0] = angular;
    output_vels[1] = linear;
    cout << "Updated angular velocity" << output_vels[0] << endl;
    cout << "Updated linear velocity" << output_vels[1] << endl;
    return output_vels;
}
