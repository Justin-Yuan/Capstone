#ifndef IMAGE_TRANSPORTER_H
#define IMAGE_TRANSPORTER_H

#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>

class imageTransporter{
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport* it_;
		image_transport::Subscriber imgSub;

		cv::Mat img;
		std::string type_;

		void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	public:
		imageTransporter(std::string topic, std::string type);
		~imageTransporter();

		cv::Mat getImg();
};

#endif
