#include "planners.h"

using namespace std;


vector<float> stupidPlanner(float x, float y, float yaw, uint8_t* bumper) {
    // Initialize the control variables
    int num_vels = 2;
    vector<float> output_vels;
    output_vels = vector<float>(num_vels, 0);
}