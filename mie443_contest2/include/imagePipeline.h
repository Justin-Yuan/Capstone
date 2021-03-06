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

// Criterias
#define MaxArea 40000.
#define MinArea 1000.
#define MaxGoodArea 60000.
#define MinGoodArea 2000.

// Sizes
#define NumTargets 3
#define NumBoxes 5
#define NumStatus 5
// #define NumViewPoints 5

// Logit update
#define alpha 2.
#define beta 1.5

class ImagePipeline
{
private:
    cv::Mat img;
    bool isValid;
    image_transport::Subscriber sub;
    vector<int> templateIDs;                           // stores final IDs
    vector<vector<float>> logits; // 1st index: boxID, 2nd index-2: target

public:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    int getTemplateID(Boxes &boxes);
    float getArea(std::vector<Point2f> scene_corners, cv::Mat img_object);
    float performSURF(cv::Mat img_scene, cv::Mat img_object);

    // To be called at each img capture
    void updateLogits(Boxes &boxes, int boxID);

    // To be called at the end
    void finalizeTemplateID(int boxID); // updates this.templateIDs from this.logits
    void finalizeTemplateIDs();         // wrapper for the finalizeTemplateID

    // Constructor
    ImagePipeline(ros::NodeHandle &n) : templateIDs(NumBoxes, -1)
    // logits(NumBoxes, vector<int>(NumStatus, 0))
    {
        image_transport::ImageTransport it(n);
        sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
        // reset all image template ids
        for (int box = 0; box < templateIDs.size(); box++)
        {
            templateIDs[box] = -1;
        }
        isValid = false;

        for (int i = 0; i < NumBoxes; i++) {
            vector<float> v(NumStatus, 0.0);
            logits.push_back(v);
        }
    }


    // Utilities
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
            name = "Empty Surface";
        }

        return name;
    }

    inline std::string box_to_name(int boxID)
    {
        return ID_to_name(box_to_ID(boxID));
    }
};
