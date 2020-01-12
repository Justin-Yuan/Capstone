#include "imageTransporter.hpp"

void imageTransporter::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		cv::Mat temp;
		temp = cv_bridge::toCvShare(msg, type_)->image;
		img = temp.clone();
	}catch (cv_bridge::Exception& e){
  		ROS_ERROR("Could not convert from '%s'", msg->encoding.c_str());
	}	
}

imageTransporter::imageTransporter(std::string topic, std::string type){
	it_ = new image_transport::ImageTransport(nh_);
	type_ = type;

	imgSub = it_->subscribe(topic, 1, &imageTransporter::imageCallback, this);
}

imageTransporter::~imageTransporter(){
	delete it_;
}

cv::Mat imageTransporter::getImg(){
	return img.clone();
}
