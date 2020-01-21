#include <cstdint>
#include <cmath>
#include <stdio.h>
#include <vector>


using namespace std;

vector<float> stupidPlanner(float x, float y, float yaw, uint8_t* bumper);

class motionPlanner {
    
public:
    // Constructor
    motionPlanner(float _x, float _y, float _yaw, uint8_t* _bumper)
    // Initialize the control variables
    int num_vels = 2;
    vector<float> output_vels;
    output_vels = vector<float>(num_vels, 0);

}