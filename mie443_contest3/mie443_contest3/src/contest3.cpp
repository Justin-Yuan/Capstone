#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	ros::Rate loop_rate(10);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	sc.playWave(path_to_sounds + "sound.wav");

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
