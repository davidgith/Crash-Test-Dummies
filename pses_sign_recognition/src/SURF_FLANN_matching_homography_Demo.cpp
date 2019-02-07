//#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

bool gui;
std::string schild = "/home/pses/catkin_ws/src/pses_sign_recognition/data/objs/6.png";

// Berechnet die Fläche eines Vierecks wenn man ihr 4 Punkte giebt
float areaQuadrangle(std::vector<Point2f> corners){
    float area = 0;

    area = 0.5 * ( corners[0].x * corners[1].y - corners[0].y * corners[1].x
                 + corners[1].x * corners[2].y - corners[1].y * corners[2].x
                 + corners[2].x * corners[3].y - corners[2].y * corners[3].x
                 + corners[3].x * corners[0].y - corners[3].y * corners[0].x);
    return area;
}


const char* keys =
        "{ help h |                          | Print help message. }"
        "{ input1 | ../data/box.png          | Path to input image 1. }"
        "{ input2 | ../data/box_in_scene.png | Path to input image 2. }";


void imageCallback(const sensor_msgs::ImageConstPtr& msg, int argc, char * argv[])
{
    static int imagecounter = 0; 
    imagecounter++;
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat imageGrey;
    cv::cvtColor(image, imageGrey, CV_BGR2GRAY);
    
    if(imagecounter == 1)
        {
        //cv::imshow("view", imageGrey);
        imagecounter = 0;
        } 

    //ROS_INFO("image callback");
    cv::waitKey(10);

    CommandLineParser parser( argc, argv, keys );
    Mat img_object = imread( parser.get<String>("input1"), IMREAD_GRAYSCALE );
    //Mat img_scene = imread( parser.get<String>("input2"), IMREAD_GRAYSCALE );
    Mat img_scene = imageGrey;

    if ( img_object.empty() || img_scene.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        parser.printMessage();
    //    return -1;
    }

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );

    //cv::imshow("descriptors_object", descriptors_object);
    //cv::imshow("descriptors_scene", descriptors_scene);

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    //ROS_INFO("knn_matches.size=%i good_matches.size=%i", (int)knn_matches.size(), (int)good_matches.size());

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

    //Only continue if there are more then 20 keypoints found
    if((int)good_matches.size() > 20){
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


        ROS_INFO("Object found! position:(%i,%i);(%i,%i);(%i,%i);(%i,%i) area:%i #keypoints=%i",(int)scene_corners[0].x , (int)scene_corners[0].y ,
                                                                                                (int)scene_corners[1].x , (int)scene_corners[1].y ,
                                                                                                (int)scene_corners[2].x , (int)scene_corners[2].y ,
                                                                                                (int)scene_corners[3].x , (int)scene_corners[3].y ,
                                                                                                (int)areaQuadrangle(scene_corners), (int)good_matches.size());
        
        // draw circles around the 4 corners
        cv::circle( img_matches, scene_corners[0] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
        cv::circle( img_matches, scene_corners[1] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
        cv::circle( img_matches, scene_corners[2] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );
        cv::circle( img_matches, scene_corners[3] + Point2f((float)img_object.cols), 5, Scalar(255, 0, 0), 2 );


        //cv::imshow("H mat", H);

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
    
    //-- Show detected matches
    imshow("Good Matches & Object detection", img_matches );


  //  waitKey();

}





int main( int argc, char* argv[] )
{
    // init this node
    ros::init(argc, argv, "SURF_FLANN_matching_homography_Demo");

    // get ros node handle
  	ros::NodeHandle nh;

      

    ROS_INFO("Node starting ...");

    // How to use parameter: http://enesbot.me/understanding-the-parameter-server-in-ros.html

    if(nh.hasParam("SURF_FLANN_matching_homography_Demo/gui")){
		ROS_INFO("gui parameter found");
		}
		else ROS_INFO("gui parameter not found");

    nh.param("SURF_FLANN_matching_homography_Demo/gui", gui, true);

    ROS_INFO("gui=%d", gui);



    if(nh.hasParam("SURF_FLANN_matching_homography_Demo/schild")){
		ROS_INFO("schild parameter found");
		}
		else ROS_INFO("schild parameter not found");

    nh.param("SURF_FLANN_matching_homography_Demo/schild", schild, schild);

    ROS_INFO_STREAM(schild);




    ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, argc, argv));


    ros::Rate loop_rate(25);
  
    while (ros::ok())
    {

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
    }
    
    return 1;
    ros::spin();
    
}
