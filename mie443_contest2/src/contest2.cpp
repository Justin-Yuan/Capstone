#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // subscribe to map server to grab map 
    Navigation nav();
    ros::Subscriber mapSub = n.subscribe("/map", 1, &Navigation::mapCallback, &nav);

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    // Get starting pose, append it to the box coordinates, and then find the traversal path.
    // vector<int> starting_pose = getStartingPose();
    std::vector<std::vector<float>> traversal_path = boxes.coords;
    // traversal_path.push_back(starting_pose);
    traversal_path = nav.getTraversalOrder(traversal_path, traversal_path.size() - 1);

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        for (i = 0; i < traversal_path.size() - 1; i++){
            nav.moveToGoal(traversal_path[i][0], traversal_path[i][1], traversal_path[i][2]);
            // run the image detection algorithm (this might include taking the image at 3 different viewpoints)
            // imagePipeline.getTemplateID(boxes); <= I have no idea how this function works, but it is here as an example.
            ros::Duration(0.01).sleep();
        }
        // Move back to start and end run.
        nav.moveToGoal(traversal_path[i][0], traversal_path[i][1], traversal_path[i][2]);
        break;
    }
    return 0;
}


"""
1. localize to determine starting position
2. determinze objecgt traversal order 
3. generate veiw points (margin is half wall size + half robot size + delta)
4. order in a sequence (optioanlly insert starting point) (this is done)
4. start traversing
5. return to start 
"""
