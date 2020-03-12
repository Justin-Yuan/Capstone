#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;
        vector<int> templateIDs(5, -1);

    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
        float getArea(std::vector<Point2f> scene_corners, cv::Mat img_object);
        float performSURF(cv::Mat img_scene, cv::Mat img_object);
        void ImagePipeline::updateTemplateID(Boxes &boxes, int boxID);

        void set_templateID(int templateID, int boxID)
        {
            templateIDs[boxID] = templateID;
        }

        int get_templateID(){
            return templateIDs(5, -1);
        }
};
