#pragma once

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <vector>
#include <map>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boxes.h>
#include <math.h>


class Navigation {
	public:
		int width;
		int height;
		float resolution;
		std::vector<float> origin;
		int[] map;
		ros::Subscriber mapSub;
		int num_view_points;
		std::map<int,std::vector<std::vector<float>>> box_view_points;
		std::vector<std::vector<float>> traj_points;

		Navigation(ros::NodeHandle &n, int n_view_points) {
			mapSub = n.subscribe("/map", 1, &Navigation::mapCallback, this);
			num_view_points = num_view_points;
		}
    
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void getViewPoints(Boxes &boxes);
		std::vector<int> getTraversalOrder(std::vector<std::vector<float>> coords, int starting_pos);
		void Navigation::traverseAllBoxes(Boxes &boxes);
		int getDist(std::vector<float> coor1, std::vector<float> coor2);
};
