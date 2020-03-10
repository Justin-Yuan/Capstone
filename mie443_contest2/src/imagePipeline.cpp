#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

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

float computeArea(std::vector<Point2f> scene_corners, cv::Mat img_object) {
    // Might need additional checks
    if (scene_corners.size() < 4) return 0.0;
    auto pointA = scene_corners[0] + Point2f( img_object.cols, 0);
    auto pointB = scene_corners[1] + Point2f( img_object.cols, 0);
    auto pointC = scene_corners[2] + Point2f( img_object.cols, 0);
    auto pointD = scene_corners[3] + Point2f( img_object.cols, 0);

    cout << pointA.x << " "<<pointA.y<<endl;
    cout << pointB.x << " "<<pointB.y<<endl;
    cout << pointC.x << " "<<pointC.y<<endl;
    cout << pointD.x << " "<<pointD.y<<endl;

    auto xMin = fmin(fmin(pointA.x, pointB.x), fmin(pointC.x, pointD.x));
    auto yMin = fmin(fmin(pointA.y, pointB.y), fmin(pointC.y, pointD.y));
    auto xMax = fmax(fmax(pointA.x, pointB.x), fmax(pointC.x, pointD.x));
    auto yMax = fmax(fmax(pointA.y, pointB.y), fmax(pointC.y, pointD.y));


    float s1 = xMax - xMin;
    float s2 = yMax - yMin;

    float area = s1 * s2;
    
    //return positive area it ressembles a rectanle, otherwise return negative and equal area
    if ((abs(pointA.x - pointD.x) < 50) && (abs(pointB.x - pointC.x) < 50) && (abs(pointA.y - pointB.y) < 50) && (abs(pointC.y - pointD.y) < 50))
        return area;
    else
        return -1*area;
}

int ImagePipeline::getTemplateID(Boxes &boxes){

    // read template images. TODO: use load templates function in boxes.cpp
    cv::Mat image_array_1 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template1.jpg",IMREAD_GRAYSCALE);
    cv::Mat image_array_2 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template2.jpg",IMREAD_GRAYSCALE);
    cv::Mat image_array_3 = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/boxes_database/template3.jpg",IMREAD_GRAYSCALE);

    int template_id = -1;
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

        // initialize 
        vector<float> rectAreas(3, 0.0);
        int checkImage = 0;

        // Get bounding box areas
        checkImage = compareImages(img, image_array_1, rectAreas[0]); //need to fix this
        cout << "IMG0-> Matches: " << checkImage << " Area: " << rectAreas[0] << endl;

        checkImage = compareImages(img, image_array_2, rectAreas[1]);
        cout << "IMG1-> Matches: " << checkImage << " Area: " << rectAreas[1] << endl;

        checkImage = compareImages(img, image_array_3, rectAreas[2]);
        cout << "IMG2-> Matches: " << checkImage << " Area: " << rectAreas[2] << endl;

        // Finding matches
        int potentialMatch = -1;
        int matchCount = 0;
        int definiteNotMatch = 0;

        for (int i = 0; i < rectAreas.size() && matchCount < 2; i++)
        {
            //If picture fits within area bounds and is a rectange, a match is found
            if (abs(rectAreas[i]) > 5000.0 && abs(rectAreas[i]) < 35000.0 && rectAreas[i] > 0)
            {
                cout << "MATCH FOUND: image " << i << std::endl;
                potentialMatch = i;
                matchCount++;
                //If picture is far away from the bounds or is not a rectangle, a definite no match is found
            }
            else if ((abs(rectAreas[i]) < 1000 || abs(rectAreas[i]) > 40000.0 && rectAreas[i] < 0))
            {
                definiteNotMatch++;
            }
        }

        template_id = (matchCount == 1 ? potentialMatch : -1);

        //If all 3 angles are definite no matches, signal the controller to label this box blank and continue
        if (definiteNotMatch == 3)
            template_id = -2;

        // Use: boxes.templates
        cv::imshow("view", img);
        cv::waitKey(10);
    }
    return template_id;
}

int compareImages(cv::Mat img_scene, cv::Mat img_object, float &area)
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
    return good_matches.size();
}

//   /** @function readme */
//   void readme()
//   { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
