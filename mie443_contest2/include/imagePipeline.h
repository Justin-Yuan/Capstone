#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <boxes.h>
#include <vector>

// Matching status
#define RAISIN 0
#define CINNAMON 1
#define RICE 2
#define AMBIGUITY -1
#define BLANK -2
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"


using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;
        std::vector<int> templateIDs{vector<int>(5,-1)};

    public:
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
        float getArea(std::vector<Point2f> scene_corners, cv::Mat img_object);
        float performSURF(cv::Mat img_scene, cv::Mat img_object);
        void updateTemplateID(Boxes &boxes, int boxID);

        ImagePipeline(ros::NodeHandle &n)
        {
            image_transport::ImageTransport it(n);
            sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
            // reset all image template ids 
            for (int i = 0; i < templateIDs.size(); i++) {
                templateIDs[i] = -1;
            }
            isValid = false;
        }

        inline void setTemplateID(int templateID, int boxID)
        {
            templateIDs[boxID] = templateID;
        }

        inline int box_to_ID(int boxID)
        {
            return templateIDs[boxID];
        }

        inline std::string ID_to_name(int templateID)
        {
            std::string name = "N/A";
            switch (templateID)
            {
            case RAISIN:
                name = "Raisin Bran";
                break;
            case CINNAMON:
                name = "Cinnamon Toast Crunch";
                break;
            case RICE:
                name = "Rice Krispies";
                break;
            case BLANK:
                name = "Empty Surface";
            default:
                name = "N/A";
            }

            return name;
        }

        inline std::string box_to_name(int boxID)
        {
            return ID_to_name(box_to_ID(boxID));
        }
};
