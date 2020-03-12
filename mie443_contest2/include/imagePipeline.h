#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>
#include <vector>

// Matching status
#define RAISIN 0
#define CINNAMON 1
#define RICE 2
#define AMBIGUITY -1
#define BLANK -2

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
        void updateTemplateID(Boxes &boxes, int boxID);

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
