#pragma once

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		std::vector<std::vector<float>> Navigation::getTraversalOrder(std::vector<std::vector<float>> coords, int starting_pos);
		int Navigation::getDist(std::vector<float> coor1, std::vector<float> coor2);
};
