#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
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


    while (robotPose.x == 0 && robotPose.y == 0 && robotPose.phi == 0){
        ros::spinOnce();
        std::cout <<"Nah"<<std::endl;
    }

    /* Params */
    float boxDistance = 0.75;

    std::vector<float> origin{robotPose.x, robotPose.y, robotPose.phi};
    std::vector<std::vector<float>> orderBoxes = findOptimalPath({robotPose.x, robotPose.y, robotPose.phi}, boxes.coords);
    std::vector<std::vector<float>> path = computeTarget(orderBoxes, boxDistance);
    int index = 0;
    int found[3] = {0};
    bool goalFound = false;
    std::ofstream f;
    f.open ("/home/hmnikola/ouputC2.txt");

    while(ros::ok() && index < path.size()) {
        ros::spinOnce();
        std::cout<<"Robot Position: " << " x: " << robotPose.x << " y: " << robotPose.y << " z: " 
            << robotPose.phi << std::endl;

        std::cout << "Curent Goal:"<<path[index][0]<< " " <<path[index][1]<< " " <<path[index][2]<<std::endl;
        goalFound = Navigation::moveToGoal(path[index][0], path[index][1], path[index][2]);
        ros::spinOnce();
        int match = imagePipeline.getTemplateID(boxes);
        std::string boxType = "none";

        switch(match) {
        case 0 :
            boxType = "Raisin Bran";
            break;
        case 1 :
            boxType = "Cinnamon Toast Crunch";
            break;
        case 2 :
            boxType = "Rice Krispies";
            break;
        default :
            boxType = "None";
        }

        if (goalFound){
            if (match >=0 && match <=2){ //found a good match, record it and skip to next box
                if (found[match] == 1){//duplicate match found
                    f << "Found duplicate of "<<boxType<<", (image " << match<<") at location ("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                }
                else { //first match of box type found
                    f << "Found "<<boxType<<", (image " << match<<") at location("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                }
                found[match] = 1;
                index += (3 - index%3);
            } else if (match == -2 || index%3 == 2){ //A definite blank box or all 3 angles couldn't find a match
                    f << "Found blank box at position ("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                    index += (3 - index%3);
            }
            else { //not a great match but not a definite blank either, try a different angle
                index++;
            }
        } else {
            std::vector<float> failedBox = {orderBoxes[index/3][0], orderBoxes[index/3][1], orderBoxes[index/3][2]};
            std::vector<std::vector<float>> failedBoxes;
            failedBoxes.push_back(failedBox);
            std::vector<std::vector<float>> newTargets = computeTarget(failedBoxes, boxDistance - 0.10);
            for (int i = 0; i < newTargets.size(); i++){
                path.push_back(newTargets[i]);
            }
            index += (3 - index%3);
        }

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
    Navigation::moveToGoal(origin[0], origin[1], origin[2]);

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
