#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

#define NumTargets 3
#define MaxArea 40000.
#define MinArea 1000.
#define MaxGoodArea 35000.
#define MinGoodArea 5000.

#define AMBIGUITY -1
#define BLANK -2

ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

float computeArea(std::vector<Point2f> scene_corners, cv::Mat img_object)
{
    if (scene_corners.size() < 4){
        cout << " scene_corners size not correct, should be 4 "<<endl;
        return 0.0;
    }  
    auto points[4] = {};
    for (int i : scene_corners.size()){
        points[i] = scene_corners[i] + Point2f( img_object.cols, 0);
        cout << points[i].x << " "<<points[i].y<<endl;
    }
    // Get corner points for rectangle
    auto x_min = fmin(fmin(points[0].x, points[1].x), fmin(points[2].x, points[3].x));
    auto y_min = fmin(fmin(points[0].y, points[1].y), fmin(points[2].y, points[3].y));
    auto x_max = fmax(fmax(points[1].x, points[1].x), fmax(points[2].x, points[3].x));
    auto y_max = fmax(fmax(points[1].y, points[1].y), fmax(points[2].y, points[3].y));

    float length = abs(x_max - x_min); 
    float width = abs(y_max - y_min);
    float area = length * width;

    // check if area is a valid rectangle, return engative value of the area if not valid
    if ((abs(points[0].x - points[3].x) < 50) && (abs(points[1].x - points[2].x) < 50) && (abs(points[0].y - points[1].y) < 50) && (abs(points[2].y - points[3].y) < 50))
        return area;
    else
        return -1 * area;
}

int ImagePipeline::getTemplateID(Boxes &boxes){
    std::string save_im_path = "";
    cv::imwrite(save_im_path, scene_transformed);

    cv::Mat target_1 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template1.jpg", IMREAD_GRAYSCALE);
    cv::Mat target_2 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template2.jpg", IMREAD_GRAYSCALE);
    cv::Mat target_3 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template3.jpg", IMREAD_GRAYSCALE);

    int templateID = -1;
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {
        /***YOUR CODE HERE***/
        // Store rectangle areas of each img-target matching in an array
        vector<float> matchedAreas(NumTargets, 0.0);
        for (int i = 0; i < NumTargets; i++)
        {
            switch (i)
            {
            case 0:
                target = target_0;
                break;
            case 1:
                target = target_1;
                break;
            case 2:
                target = target_2;
                break;
            }
            matchedAreas[i] = performSURF(img, target);
            cout << "Target " << i << " | area = " << matchedAreas[i] << endl;
        }

        // Examine each matching area and decide which one (or none) to be chosen
        int candidateID = -1, candidateCount = 0, antiCandidateCount = 0;
        for (int i = 0; i < NumTargets && candidateCount < 2; i++)
        {
            float area = abs(matchedAreas[i]);
            bool isRectangle = (matchedAreas[i] > 0);
            bool isGoodSized = (area < MaxGoodArea && area > MinGoodArea);
            bool isOutofBound = (area > MinArea || area < MinArea);

            if (isRectangle && isGoodSized)
            {
                candidateID = i;
                candidateCount++;
                cout << "\n--- Matching target " << i << "---\n";
            }
            else if (!isRectangle && isOutofBound)
            {
                antiCandidateCount++;
            }
        }

        switch (candidateCount)
        {
        case 1: // one good match found
            templateID = candidateID;
            break;
        case 0: // other cases
        case 2:
        case 3:
            templateID = AMBIGUITY;
            break;
        default: // certain that all three are non-matches
            if (antiCandidateCount == 3) templateID = BLANK;
        }

        // Use: boxes.templates
        cv::imshow("view", img);
        cv::waitKey(10);
    }
    return templateID;
}

float performSURF(cv::Mat img_scene, cv::Mat img_object)
{
    //-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute(img_object, Mat(), keypoints_object,
                               descriptors_object);
    detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
                               descriptors_scene);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors_object, descriptors_scene, matches);

    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<DMatch> good_matches;

    for (int i = 0; i < descriptors_object.rows; i++)
    {
        if (matches[i].distance < 3 * min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_matches;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    if (obj.size() < 4 || scene.size() < 4)
    {
        return 0;
    }
    Mat H = findHomography(obj, scene, RANSAC);

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
    obj_corners[3] = cvPoint(0, img_object.rows);
    std::vector<Point2f> scene_corners(4);

    cout << "OBJ: " << obj.size() << endl;
    cout << "SCENE: " << scene.size() << endl;

    if (H.empty())
        return 0;

    perspectiveTransform(obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

    area = computeArea(scene_corners, img_object);

    //-- Show detected matches
    imshow("Good Matches & Object detection", img_matches);

    waitKey(5000);
    return area;
}