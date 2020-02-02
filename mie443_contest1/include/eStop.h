#include <sensor_msgs/Joy.h>

class teleController{
public:	
	teleController(){
		linear = 0.0;
		angular = 0.0;
		state = 0;
		teleop = 0;
	}

	void controllerCallback(const sensor_msgs::Joy::ConstPtr& joy){
        	linear = 0.2*joy->axes[1];
        	angular = 1.2*joy->axes[0];

		if(joy->buttons[0] == 1 && state == 0){
			state = 1;
		}

		if(joy->buttons[1] == 1 && state == 1){
			state = 0;
		}

		if(joy->buttons[2] == 1){
			teleop = !teleop; 
		}
			
	}
    void block(){
        while(state == 1){};
    }

	double getLinear(){
		return linear;
	}

	double getAngular(){
		return angular;
	}

	bool getState(){
		return state;
	}

	bool getTeleop(){
		return teleop;
	}

	void setLinear(const double &i){
		linear = i;
	}

	void setAngular(const double &i){
		angular = i; 
	}

	void setState(const bool &i){
		state = i;
	}


private:
	double linear, angular;
	bool state;
    bool teleop;
};

