// #include "mie443_contest1/include/planners.h"
#include "planners.h"

vector<float> motionPlanner::simpleWallFollowing()
{
    cout << "Inprogress" << endl;
}

// vector<float> motionPlanner::tutorialPlanner()
void motionPlanner::tutorialPlanner()
{
    cout << "in function" << endl;
    // float angular = 0.0;
    // float linear = 0.0;

    // // Check if any of the bumpers were pressed.
    // bool any_bumper_pressed = false;
    // bool pressed = false;
    // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
    //     pressed = (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    //     any_bumper_pressed |= pressed;
    //     ROS_INFO("Bumper: (%i) Pressed: %s", b_idx, pressed?"true":"false");
    // }
    // // Control logic after bumpers are being pressed.
    // //
    // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
    // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
    //     angular = 0.0;
    //     linear = 0.2;
    // }
    // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
    //     angular = M_PI / 6;
    //     linear = 0.0;
    // }
    // else if (minLaserDist > 1. && !any_bumper_pressed) {
    //     linear = 0.1;
    //     if (yaw < 17 / 36 * M_PI || posX > 0.6) {
    //         angular = M_PI / 12.;
    //     }
    //     else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
    //         angular = -M_PI / 12.;
    //     }
    //     else {
    //         angular = 0;
    //     }
    // }
    // else {
    //     angular = 0.0;
    //     linear = 0.0;
    // }

    // output_vels[0] = angular;
    // output_vels[1] = linear;
    // cout << "Updated angular velocity" << output_vels[0] << endl;
    // cout << "Updated linear velocity" << output_vels[1] << endl;
    // return output_vels;
    // return output_vels;
    return;
}
