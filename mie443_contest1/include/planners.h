#ifndef PLANNERS_HEADER
#define PLANNERS_HEADER

#include <cstdint>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace std;


class motionPlanner {
// If we really want, we can have another "stupidPlanner" class inherenting from this one
private:
    const float x, y, yaw;
    const uint8_t *bumper;
    const float minLaserDist;

public :
    // Initialize the return
    vector<float> output_vels;

    // Constructor
    motionPlanner(float _x, float _y, float _yaw, float _laser, uint8_t *_bumper) : x(_x), y(_y), yaw(_yaw), minLaserDist(_laser), bumper(_bumper)
    {
        // Initialize the control variables
        int num_vels = 2;
        output_vels = vector<float>(num_vels, 0);
    }

    ~motionPlanner();

    // Functions
    vector<float> dummyfunc(float x, float y, float yaw, uint8_t *bumper);
};


#endif