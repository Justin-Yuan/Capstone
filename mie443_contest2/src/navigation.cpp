#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){
	// Set up and wait for actionClient.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
	// Set goal.
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  xGoal;
    goal.target_pose.pose.position.y =  yGoal;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = phi.z;
    goal.target_pose.pose.orientation.w = phi.w;
    ROS_INFO("Sending goal location ...");
	// Send goal and wait for response.
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}

std::vector<std::vector<float>> Navigation::getTraversalOrder(std::vector<std::vector<float>> coords, int starting_pos){
    // Modified travelling salesman problem solution from: https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/

    // store all vertex apart from source vertex 
    std::vector<std::vector<float>> vertex; 
    for (int i = 0; i < V; i++) 
        if (i != starting_pos) 
            vertex.push_back(coords[i]); 
  
    // store minimum weight Hamiltonian Cycle. 
    int min_path = INT_MAX; 
    std::vector<std::vector<float>> min_vertex = vertex;
    do { 
        // store current Path weight(cost) 
        int current_pathweight = getDist(coords[starting_pos], vertex[0]); 
          
        // compute current path weight 
        for (int i = 0; i < vertex.size() - 1; i++) { 
            current_pathweight += getDist(vertex[i], vertex[i+1]);
        } 
        current_pathweight += getDist(vertex[i], coords[starting_pos]); 
  
        // update minimum 
        if (current_pathweight < min_path){
            min_path = current_pathweight; 
            min_vertex = vertex
        }
        
    } while (next_permutation(vertex.begin(), vertex.end())); 
    
    // Append starting coord to end of vector.
    vertex.push_back(coords[starting_pos])
    return vertex;
}

int Navigation::getDist(std::vector<float> coor1, std::vector<float> coor2){
    return pow(pow((coor1[0] - coor2[0]),2) + pow((coor1[1] - coor2[1]),2),1/2);
}
