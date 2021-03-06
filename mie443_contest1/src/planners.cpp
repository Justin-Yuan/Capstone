// #include "mie443_contest1/include/planners.h"
#include "planners.h"

/* Public Functions */

void motionPlanner::startup()
{
    ROS_INFO("Performing Startup");
    chooseDirection();
}

void motionPlanner::step()
{
    ROS_INFO("Stepping");
    ros::spinOnce();
    plannerMain();
}


/* Private Functions */

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Planning functions //////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motionPlanner::plannerMain()
{
    float angular, linear;
    // Offset Calculation
    int left_index = laserSize - 1 - laserOffset, right_index = laserOffset;
    int minLeftLaserIndex = left_index - ((laserSize - 1) / 2);
    int minRightLaserIndex = ((laserSize - 1) / 2) - right_index;

    // Reevaluate the state every certain duration
    if (time_passed - time_last_update >= time_step) setState();

    ROS_INFO("Current State: %d, LeftRange: %f, RightRange: %f", state, minLeftLaserDist, minRightLaserDist);

    // Choose direction if in exploration state
    if (state == EXPLORE){
        if (dist(prevX, prevY, posX, posY) > explore_per_dist)
        {
            prevX = posX;
            prevY = posY;
            chooseDirection();
        }
    }

    // Check and then start moving if everything is fine
    ros::spinOnce();
    eStop.block();
    checkBumpers();


    // -------------------------------------------------------------------------------------
    // ---------- START OF MAIN CONTROL LOGIC ----------------------------------------------
    // -------------------------------------------------------------------------------------

    if (minLaserDist > obstacleDist+obstacleDist_zone)
    {
        if (state = EXPLORE)
        {
            angular = stayCentered(minLeftLaserDist, minRightLaserDist, minLeftLaserIndex, minRightLaserIndex, k_p_small, 0.);
        }
        else {
            angular = 0.;
        }

        linear = linear_max;
        angular = stayAwayFromWalls(minLeftLaserDist, minRightLaserDist, angular);
    }

    // When the front sensor reading is too low
    else if (minLaserDist < obstacleDist-obstacleDist_zone)
    {
        // Determine which side has more space
        if (minRightLaserDist < minLeftLaserDist) {
            rotate2explore();
        }
        else {
            rotate2explore(CW);
        }
    }

    // When you are in the safe but not thaaaat safe zone, so turn faster and also be ready to slow down
    else
    {
        angular = stayCentered(minLeftLaserDist, minRightLaserDist, 0., 0., k_p_big, 0.);
        linear = stayChill(minLaserDist);
    }

    // -------------------------------------------------------------------------------------
    // ---------- END OF MAIN CONTROL LOGIC ------------------------------------------------
    // -------------------------------------------------------------------------------------


    // write the defined speed to the robot
    ROS_INFO("Main Publishing Velocity");
    publishVelocity(angular, linear, true /* SpinOnce */);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bumper Chdck ////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motionPlanner::checkBumpers() {
    if (bumper[kobuki_msgs::BumperEvent::LEFT] == 1 ||
        bumper[kobuki_msgs::BumperEvent::CENTER] == 1 ||
        bumper[kobuki_msgs::BumperEvent::RIGHT] == 1)
    {
        float startX, startY;
        bool right = bumper[kobuki_msgs::BumperEvent::RIGHT] == 1;
        bool center = bumper[kobuki_msgs::BumperEvent::CENTER] == 1;

        // Maneuver backwards
        startX = posX; startY = posY;
        while (dist(startX, startY, posX, posY) < bumperPullbackDist)
        {
           publishVelocity(0 /* angular */, -0.1 /* linear */, true /* spinOnce */);
        }

        // If it's the front bumper, redirect the robot
        if (center)
        {
            chooseDirection();
        }
        // If side, do a "__/" kind of reruoting
        else
        {
            // Adjust angle away from obstacle
            if (right)
                rotate2angle(20);
            else
                rotate2angle(20, CW);

            // Maneuver forwards
            startX = posX;
            startY = posY;
            while (dist(startX, startY, posX, posY) < 0.15)
            {
                publishVelocity(0 /* angular */, 0.1 /* linear */, true /* spinOnce */);
            }

            // Adjust angle back to original direction
            if (right)
                rotate2angle(20, CW);
            else
                rotate2angle(20);
        }
    }
}

geometry_msgs::Twist motionPlanner::threeRegion() {
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
// Rotation ////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool motionPlanner::inRange(int bin, const vector<double> & binRange, bool front /* = false */){
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

void motionPlanner::rotate2angle(float angle, bool CCW /* = true */) {
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
    double rad = DEG2RAD(angle); // TODO: maybe move it to before passing to rotate

    // rotate until desired
    while (abs(yaw - currYaw) < rad)
    {
        publishVelocity(angular, 0.0, true /* SpinOnce */);
    }
}

void motionPlanner::rotate2explore(bool CCW /* = true */) {
    /**
     * Rotate the robot until it is heading to a further wall/object
     * @param  {CCW} bool : default rotation is CCW == turn left, set to false if need to turn right
     */
    double angular = angular_max;
    if (!CCW)
        angular = angular * -1;

    // Stop turning (ready to go forward linearly) if there's something far away enough
    while (minLaserDist < exploreDist || minLeftLaserDist < exploreDist_lr || minRightLaserDist < exploreDist_lr)
    {
        publishVelocity(angular, 0.0, true /* SpinOnce */);
    }
}

void motionPlanner::rotate2bin(int bin) {
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

void motionPlanner::chooseDirection() {
    /**
     * Rotate and choose direction to explore
     */
    // Define variables to store maximum value
    float maxDist_front = 0., maxDist_side = 0.; // default is 0 zo that if things go weird, it just moves forward
    int maxDist_front_idx = 0, maxDist_side_idx = 0;

    // Explore the front/left/right zones (no back zone!)
    for (int bin = 0; bin < exploreAngle_bins; bin++)
    {
        ros::spinOnce();

        // TODO: logic is fine, but might need to change the code appearance
        if (inRange(bin, exploreZone_front, FRONT))
        {
            if (minLaserDist > maxDist_front)
            {
                maxDist_front = minLaserDist;
                maxDist_front_idx = bin;
            }
        }
        else if (inRange(bin, exploreZone_left) || inRange(bin, exploreZone_right))
        {
            if (minLaserDist > maxDist_side)
            {
                maxDist_side = minLaserDist;
                maxDist_side_idx = bin;
            }
        }
        rotate2angle(exploreAngle_size);
    }

    if (maxDist_front < obstacleDist && maxDist_side < obstacleDist_side)
    { // Turn back around if its a dead end
        rotate2angle(turnBack);
    }
    else if (maxDist_side > exploreDist_side)
    { // Prefers to turn to the side in this state
        rotate2bin(maxDist_side_idx);
    }
    else
    { // If neither, go forward
        rotate2bin(maxDist_front_idx);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adjustment functions ////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
float motionPlanner::stayAwayFromWalls(float leftDist, float rightDist, float default_angular) {
    if (leftDist < 0.5) {
        return -angular_max;
    }
    else if (rightDist < 0.5) {
        return angular_max;
    }
    else {
        return default_angular;
    }
}

float motionPlanner::stayCentered(float leftDist, float rightDist, int leftIndex, int rightIndex, float k_p, float default_angular){
    float curr_diff_lr = leftDist - rightDist;
    int curr_diff_index = leftIndex - rightIndex;

    float angular = default_angular;

    // Distance difference can't be too large
    if (curr_diff_lr > allowed_laser_diff_lr) {
        angular = k_p * curr_diff_lr / leftDist;
    }
    else if (-curr_diff_lr > allowed_laser_diff_lr) {
        angular = k_p * curr_diff_lr / rightDist;
    }

    // Index difference can't be too large either (otherwise orientation is skewed)
    if (curr_diff_index > allowed_laser_diff_index)
        angular = -angular_max;
    else if (-curr_diff_index > allowed_laser_diff_index)
        angular = angular_max;

    return angular;
}

float motionPlanner:: stayChill(float frontDist){
        float k_p_linear = 0.8; // even i
        float dangerousDist = frontDist - obstacleDist; // if < 0, dangerous

        if (dangerousDist <= -0.5 * obstacleDist_zone) {
            k_p_linear = 0.3;
        }
        else if (dangerousDist <= obstacleDist_zone) {
            k_p_linear = 0.5;
        }
        else if (dangerousDist <= 0) {
            k_p_linear = 0.65;
        }
        else if (dangerousDist <= 0.5 * obstacleDist_zone) {
            k_p_linear = 0.7;
        }

        return k_p_linear * linear_max;
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
    ROS_INFO("Publishing - Linear: %f, Angular: %f", linear, angular);
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
// State decision  //////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motionPlanner::setState() {
    /**
     * Choose a state depending on the time passed, all variables are global
     */
    std::random_device device;
    std::mt19937 gen(device());

    ros::spinOnce();
    time_passed =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - time_start).count();
    random_prob = time_passed / time_total;

    std::bernoulli_distribution randomOrNot(random_prob);
    goRandom = randomOrNot(gen);
    if (goRandom) {
        state = EXPLORE;
    } else {
        state = FORWARD;
    }
    time_last_update = time_passed;

    ROS_INFO("%f seconds, state: %d, random_output: %d", (float) time_last_update, state, goRandom);
    return;
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
    laserOffset = desiredAngle * M_PI / (180 * msg->angle_increment);
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
