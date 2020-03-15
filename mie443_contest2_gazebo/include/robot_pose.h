#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

class RobotPose {
	public:
		float x;
		float y;
		float phi;
	public:
		RobotPose(float x, float y, float phi);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
		std::vector<float> toCoords() {
			std::vector<float> coord{x,y,phi};
			return coord;
		}

};
