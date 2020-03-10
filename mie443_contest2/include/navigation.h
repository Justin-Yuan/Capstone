#pragma once

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <vector>
#include <map>


class Navigation {
	public:
		uint32 width;
		uint32 height;
		float32 resolution;
		geometry_msgs::Pose origin;
		int8[] map;
		ros::Subscriber mapSub;
		int n_view_points;
		std::map<int,std::vector<std::vector<float>>> view_points;

		Navigation(ros::NodeHandle &n, int n_view_points) {
			mapSub = n.subscribe("/map", 1, &Navigation::mapCallback, this);
			n_view_points = n_view_points;
		}
    
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		std::vector<std::vector<float>> Navigation::getTraversalOrder(std::vector<std::vector<float>> coords, int starting_pos);
		int Navigation::getDist(std::vector<float> coor1, std::vector<float> coor2);
};
