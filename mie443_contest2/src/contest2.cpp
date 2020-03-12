#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>


#define RAISIN 0
#define CINNAMON 1
#define RICE 2
#define BLANK -2

int main(int argc, char **argv)
{
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // subscribe to map server to grab map (only once)
    Navigation nav(&n, 3);

    // Robot pose object + subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
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
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    while (robotPose.x == 0 && robotPose.y == 0 && robotPose.phi == 0)
    {
        ros::spinOnce();
        std::cout << "Nah" << std::endl;
    }

    /* Params */
    // float boxDistance = 0.75;

    // std::vector<float> origin{robotPose.x, robotPose.y, robotPose.phi};
    // std::vector<std::vector<float>> orderBoxes = findOptimalPath({robotPose.x, robotPose.y, robotPose.phi}, boxes.coords);
    // std::vector<std::vector<float>> path = computeTarget(orderBoxes, boxDistance);
    // int index = 0;
    // int matchStatus[3] = {0}; // records of this ID is already found
    // bool atBox = false;
    // std::ofstream f;
    // f.open("/home/hmnikola/ouputC2.txt");

    while (ros::ok())
    // while (ros::ok() && index < path.size())
    {
        ros::spinOnce();
        // std::cout << "Robot Position: "
        //           << " x: " << robotPose.x << " y: " << robotPose.y << " z: "
        //           << robotPose.phi << std::endl;

        // std::cout << "Curent Goal:" << path[index][0] << " " << path[index][1] << " " << path[index][2] << std::endl;
        // atBox = Navigation::moveToGoal(path[index][0], path[index][1], path[index][2]);
        // ros::spinOnce();

        // int templateID = imagePipeline.getTemplateID(boxes);
        // std::string match = "N/A";

        // switch (templateID)
        // {
        // case RAISIN:
        //     match = "Raisin Bran";
        //     break;
        // case CINNAMON:
        //     match = "Cinnamon Toast Crunch";
        //     break;
        // case RICE:
        //     match = "Rice Krispies";
        //     break;
        // case BLANK:
        //     match = "Empty Surface";
        // default:
        //     match = "N/A";
        // }

        // //-- When we're in front of a box
        // if (atBox)
        // {
        //     //-- if a certain match is found, log it's status and location --
        //     if (match != "None")
        //     {
        //         //-- check/update duplication status
        //         if (matchStatus[templateID] == 1) {
        //             f << "(Duplicate) ";
        //         } else {
        //             matchStatus[templateID] = 1;
        //         }

        //         // TODO: use the location format from Justin and Gary
        //         f match << " (ID=" << templateID << ") found at location(" << orderBoxes[index / 3][0] << ", "
        //             << orderBoxes[index / 3][1] << ", " << orderBoxes[index / 3][2] << ")" << std::endl;

        //         // TODO: change this: index += (3 - index % 3);
        //     }
        //     //-- otherwise, do whatever is needed --
        //     else
        //     {
        //         index++;
        //     }
        // }
        // When we're not at a box
        /*
        else
        {
            std::vector<float> failedBox = {orderBoxes[index / 3][0], orderBoxes[index / 3][1], orderBoxes[index / 3][2]};
            std::vector<std::vector<float>> failedBoxes;
            failedBoxes.push_back(failedBox);
            std::vector<std::vector<float>> newTargets = computeTarget(failedBoxes, boxDistance - 0.10);
            for (int i = 0; i < newTargets.size(); i++)
            {
                path.push_back(newTargets[i]);
            }
            index += (3 - index % 3);
        }

    // Navigation::moveToGoal(origin[0], origin[1], origin[2]);
        */
// 
        // Our code, reference code is commented above due to not compiling (** please ensure code compiles before adding in reference **):
        // Get starting pose, append it to the box coordinates, and then find the traversal path.
        std::vector<float> starting_pos{robotPose.x, robotPose.y, robotPose.phi};
        nav.origin = starting_pos;
        
        int i;
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

/*
1. localize to determine starting position
2. determinze objecgt traversal order
3. generate veiw points (margin is half wall size + half robot size + delta)
4. order in a sequence (optioanlly insert starting point) (this is done)
4. start traversing
5. return to start 
*/
