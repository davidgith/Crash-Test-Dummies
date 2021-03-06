/**
 * @file "sign_detection_node.cpp"
 * @brief Sign Detection node, to find road signs an publish them on ros topics.
 *
*/
// Code is based on the OpenCV-Example "SURF_FLANN_matching_homography_Demo.cpp"

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

bool gui;
ros::Publisher stopPublisher;
ros::Publisher lanePublisher;
ros::Publisher speedPublisher;

/**
* Calculates the area of a quadrilateral if you give it a vector with 4 corner points.
* @param corners a vector of Point2f with length 4
* @return the area of the quadrilateral
*/
float areaQuadrangle(std::vector<Point2f> corners){
    float area = 0;

    area = 0.5 * ( corners[0].x * corners[1].y - corners[0].y * corners[1].x
                 + corners[1].x * corners[2].y - corners[1].y * corners[2].x
                 + corners[2].x * corners[3].y - corners[2].y * corners[3].x
                 + corners[3].x * corners[0].y - corners[3].y * corners[0].x);
    return area;
}

/**
* Makes a decision, based on the coordinates of the corner points, whether a sign was found or not.
* Reduces thereby the false positive rate
* @param area the area of the quadrilateral 
* @param corners a vector of Point2f with length 4
* @return true if a sign is found
*/
bool signFound(float area, std::vector<Point2f> corners){
    if(area <= 20 || area >= 290000)
        return false;
    if( corners[1].x - corners[0].x < 10 || corners[2].x - corners[3].x < 10 ||
        corners[2].y - corners[1].y < 10 || corners[3].y - corners[0].y < 10 )
        return false;
    else return true;
}

/**
* Calculates the distance to the sign with the area of a quadrilateral.
* @param area the area of the quadrilateral
* @return the distance to the road sign in meter
*/
float distanceSign(float area){
    float distance = 385 * std::pow(area, -0.6385);
    return distance;
}

/**
* Called every time a new Kinect image enters.
* Does the object recognition.
* @param msg  
* @param argc not used
* @param argv[] not used
* @param schilder Vector of mat which contains the reference signs. 
* @param detect_thresh vector of threshold value for each sign of how many matches it counts as detected.
* @return true if a sign is found
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg, int argc, char * argv[], std::vector<Mat> schilder, std::vector<int> detect_thresh)
{
    //-- Loading kinect image (Resolution: 960x540)
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat imageGrey;
    cv::cvtColor(image, imageGrey, CV_BGR2GRAY);
    cv::waitKey(10);
    Mat img_scene = imageGrey;

    //-- Check if kinect image is found 
    if ( img_scene.empty() )
        ROS_ERROR("Could not find kinect image");
 

    //-- Step 1a: Detect the keypoints, in the camera image, using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
     
    std::vector<KeyPoint> keypoints_scene;
    
    Mat descriptors_scene;
    
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );

    //-- For each sign, check if it is found in the scene.
    // i=0 -> stop sign     i=1 -> Change lane sign     i=2 -> Speed limit sign
    for( int i = 0; i < detect_thresh.size(); i++){
        
        //-- Skip if sign referene image is not found
        if(schilder[i].empty()){
            continue;
        }

        //-- Step 1b: Detect the keypoints, in the road signs, using SURF Detector, compute the descriptors
        Mat img_object = schilder[i];
        std::vector<KeyPoint> keypoints_object;
        Mat descriptors_object;
        detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );


        //-- Step 2: Matching descriptor vectors with a FLANN based matcher
        // Since SURF is a floating-point descriptor NORM_L2 (Euclidian Distance) is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

        //-- Step 3: Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.75f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }


        //-- Draw matches
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        Mat H;

        //-- Continue only if enough key point matches have been found.
        // Threshold values are defined individually for each sign in detect_thresh
        if((int)good_matches.size() > detect_thresh[i]){
                H = findHomography( obj, scene, RANSAC );  
            } 

        if (! H.empty()){

            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0, 0);
            obj_corners[1] = Point2f( (float)img_object.cols, 0 );
            obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
            obj_corners[3] = Point2f( 0, (float)img_object.rows );
            std::vector<Point2f> scene_corners(4);
            

            perspectiveTransform( obj_corners, scene_corners, H);            
            
            float sign_area = areaQuadrangle(scene_corners);
            float sign_distance = distanceSign(sign_area);
            
            //-- Continue only if a road sign is found
            if(signFound(sign_area, scene_corners)){

                ROS_INFO("Object %i found! position:(%i,%i);(%i,%i);(%i,%i);(%i,%i) area:%i distance:%.2fm #keypoints=%i",i,
                        (int)scene_corners[0].x , (int)scene_corners[0].y ,
                        (int)scene_corners[1].x , (int)scene_corners[1].y ,
                        (int)scene_corners[2].x , (int)scene_corners[2].y ,
                        (int)scene_corners[3].x , (int)scene_corners[3].y ,
                        (int)sign_area, sign_distance, (int)good_matches.size());

                //-- Publish distance to rostopics
                std_msgs::Int32 distance;
                distance.data = (int)(sign_distance*100);
                if (i == 0) {
                    stopPublisher.publish(distance);
                } else if (i == 1) {
                    lanePublisher.publish(distance);
                } else if (i == 2) {
                    speedPublisher.publish(distance);
                }


                if(gui){

                //-- Draw circles around the 4 corners
                cv::circle( img_matches, scene_corners[0] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
                cv::circle( img_matches, scene_corners[1] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
                cv::circle( img_matches, scene_corners[2] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
                cv::circle( img_matches, scene_corners[3] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );

                //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
                    scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
                line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
                    scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
                    scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
                    scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                }
            } 
        }
        
        if (gui){
            //-- Show detected matches

            if(i==0)
                imshow("Schildererkennung Stop", img_matches );
            if(i==1)
                imshow("Schildererkennung Lane", img_matches );
            if(i==2)
                imshow("Schildererkennung Speed", img_matches );
            }
    }
}





int main( int argc, char* argv[] )
{
    // init this node
    ros::init(argc, argv, "sign_detection_node");

    // get ros node handle
  	ros::NodeHandle nh;
    
    //-- Topics to publish if signs found
    stopPublisher =
        nh.advertise<std_msgs::Int32>("/sign_detection_node/StopSign", 1);
    lanePublisher =
        nh.advertise<std_msgs::Int32>("/sign_detection_node/LaneSign", 1);
    speedPublisher =
        nh.advertise<std_msgs::Int32>("/sign_detection_node/SpeedSign", 1);
    

    ROS_INFO("Node starting ...");


    //-- The GUI can be turned on and off with the ROS-Parameter 'gui'
    if(nh.hasParam("sign_detection_node/gui")){
		ROS_INFO("gui parameter found");
		}
		else ROS_INFO("gui parameter not found");

    nh.param("sign_detection_node/gui", gui, true);

    ROS_INFO("gui=%d", gui);

    //-- Load reference road signs as greyscale 
    // index 0 -> Stop sign
    // index 1 -> Change lane sign
    // indey 2 -> Speed limit sign
    std::vector<Mat> schilder(3);
    ROS_INFO("Loading reference road signs ...");
    schilder[0] = imread( "/home/pses/catkin_ws/src/pses_sign_recognition/data/objs/stop.png", IMREAD_GRAYSCALE);
    schilder[1] = imread( "/home/pses/catkin_ws/src/pses_sign_recognition/data/objs/change_lane.png", IMREAD_GRAYSCALE);
    schilder[2] = imread( "/home/pses/catkin_ws/src/pses_sign_recognition/data/objs/50.png", IMREAD_GRAYSCALE);

    for(int i = 0; i < schilder.size(); i++){
        if(schilder[i].empty())
        ROS_WARN("Schild %i konnte nicht geladen werden", i);
    }

    //-- Threshold value for each sign of how many matches it counts as detected.
    std::vector<int> detect_thresh = {25, 13, 30};


    ROS_INFO("Waiting for kinect picture ...");

    ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, argc, argv, schilder, detect_thresh));

    ros::Rate loop_rate(10);
  
    while (ros::ok())
    {

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
    }

    ROS_INFO("After loop");
    
    return 1;
    ros::spin();
    
}
