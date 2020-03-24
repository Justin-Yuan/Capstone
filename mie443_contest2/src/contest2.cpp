#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>

// Matching status
#define RAISIN 0
#define CINNAMON 1
#define RICE 2
#define AMBIGUITY -1
#define BLANK -2

int main(int argc, char **argv)
{
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Initialize box coordinates and templates
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }

    // nav contains subscriber to map_server, amcl and imagePipeline
    std::cout << "Loading navigation: " << std::endl;
    Navigation nav(n, boxes, 3);

    // Execute strategy.
    // while (ros::ok())
    // {
        std::cout << "Beginning navigation: " << std::endl;
        nav.traverseAllBoxes();
    // }
    return 0;
}
