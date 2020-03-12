#include <navigation.h>

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

void Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    ROS_INFO("Map: (%d, %d) retrieved", width, height);
    // only get map once
    // mapSub.unregister();
    mapSub.shutdown();
}

void Navigation::getViewPoints(std::vector<std::vector<float>> coords) {
    float margin = 10;
    int i = 0;

    for(auto b: coords) {
        std::vector<std::vector<float>> view_points;
        float x = b[0];
        float y = b[1];
        float angle = b[2];
        
        // generate view points
        float ang_delta = M_PI / (num_view_points + 1);
        for (int i = 0; i < num_view_points; i++) {
            float view_ang = angle - M_PI/2.0 + (i+1)*ang_delta;
            float view_x = x + margin * cos(view_ang);
            float view_y = y + margin * sin(view_ang);

            std::vector<float> view_pose{view_x, view_y, view_ang};
            view_points.push_back(view_pose);
        }

        // add to current box 
        box_view_points.insert(
            std::pair<int,std::vector<std::vector<float>>>(
                i, view_points
        ));
        i++;
    }
}


std::vector<int> Navigation::getTraversalOrder(std::vector<std::vector<float>> coords, int starting_pos){
    // Modified travelling salesman problem solution from: https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/

    // store all vertex apart from source vertex 
    std::vector<int> vertex; 
    for (int i = 0; i < coords.size(); i++) 
        if (i != starting_pos) 
            vertex.push_back(i); 
  
    // store minimum weight Hamiltonian Cycle. 
    int min_path = INT_MAX; 
    std::vector<int> min_vertex = vertex;
    do { 
        // store current Path weight(cost) 
        int current_pathweight = getDist(coords[starting_pos], coords[vertex[0]]); 
          
        // compute current path weight 
        for (int i = 0; i < vertex.size() - 1; i++) { 
            current_pathweight += getDist(coords[vertex[i]], coords[vertex[i+1]]);
        } 
        current_pathweight += getDist(coords[vertex[vertex.size()-1]], coords[starting_pos]); 
  
        // update minimum 
        if (current_pathweight < min_path){
            min_path = current_pathweight; 
            min_vertex = vertex;
        }
        
    } while (next_permutation(vertex.begin(), vertex.end())); 
    
    // Append starting coord to end of vector.
    vertex.push_back(starting_pos);
    return vertex;
}

int Navigation::getDist(std::vector<float> coor1, std::vector<float> coor2){
    return pow(pow((coor1[0] - coor2[0]),2) + pow((coor1[1] - coor2[1]),2),1/2);
}

void Navigation::localizeStartingPose() {
    ros::spinOnce();
    std::vector<float> starting_pos{robotPose.x, robotPose.y, robotPose.phi};
    origin = starting_pos;
}

int Navigation::getCurrentBoxId() {
    // get nearest box as current 
    std::vector<float> curr_pos = robotPose.toCoords();
    int box_id = -1;
    float curr_dist = 10000;

    for (int i = 0; i < boxes.coords.size(); i++) {
        float dist = getDist(curr_pos, boxes.coords[i]);
        if (dist < curr_dist) {
            box_id = i;
            curr_dist = dist;
        }
    }
    return box_id;
}

void Navigation::traverseAllBoxes() {
    // set up starting pose
    localizeStartingPose();

    // set up all view points first 
    getViewPoints(boxes.coords);

    // determine box traversal order 
    std::vector<std::vector<float>> traversal_nodes = boxes.coords;
    traversal_nodes.push_back(origin);
    std::vector<int> indices = getTraversalOrder(traversal_nodes, traversal_nodes.size()-1);

    // traverse every box and then return to starting point
    for (int i = 0; i < indices.size() - 1; i++) {
        traverseBox(indices[i]);
        // periodic log 
        logImageIDs();
    }
    moveToGoal(origin[0], origin[1], origin[2]);
    logImageIDs();
}

void Navigation::traverseBox(int box_idx) {
    // traverse the given box from current position 
    std::map<int,std::vector<std::vector<float>>>::iterator it = box_view_points.find(box_idx); 
    if(it == box_view_points.end()) {
        ROS_INFO("Cannot find box with given index...");
    }
    else {
        // get current position and current image 
        ros::spinOnce();
        std::vector<float> curr_pos = robotPose.toCoords();

        // determine forward or backward traversal order
        std::vector<std::vector<float>> view_points = it->second;
        int start_idx = 0;
        int end_idx = view_points.size()-1; 
        int step = 1;

        float dist_first = getDist(curr_pos, view_points[start_idx]);
        float dist_last = getDist(curr_pos, view_points[end_idx]);
        if (dist_first > dist_last) {
            start_idx = view_points.size()-1;
            end_idx = 0;
            step = -1;
        }

        // start traversing 
        int curr_idx = start_idx;
        while (curr_idx != end_idx) {
            std::vector<float> curr_goal = view_points[curr_idx];
            // move to veiw point 
            moveToGoal(curr_goal[0], curr_goal[1], curr_goal[2]);
            // do image stuff 
            imagePipeline.updateTemplateID(boxes, box_idx);
            // next view point 
            curr_idx += step;
        } 
        ROS_INFO("Traversed box %d", box_idx);
    }
}


void Navigation::logImageIDs() {
    // load current image recognition progress
    for (int i = 0; i < boxes.coords.size(); i++) {
        int img_id = imagePipeline.box_to_ID(i);
        std::string img_name = imagePipeline.ID_to_name(img_id);
        float x = boxes.coords[i][0];
        float y = boxes.coords[i][1];
        ROS_INFO("Box %d is image %d (%s) at (%f, %f)", i, img_id, img_name.c_str(), x, y);
    }
}
