#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <iostream>
#include <stdio.h>

#include "IPM.h"

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control_deviation)
{
  try
  {
    // setup time
    clock_t begin = clock();

    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat HSVImage;
    cvtColor(image,HSVImage,CV_BGR2HSV);
    ROS_INFO("Received new image!");

    // filter green
    cv::Mat ThreshImage;
    inRange(HSVImage,cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImage);
    cv::imshow("greenfilter", ThreshImage);
    ROS_INFO("Shown new image! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
    //cv::waitKey(30);

    // Find control deviation in pixels (approximately linear?)
    for (int w = ThreshImage.cols-1; w >= 0; w--) {
      if (ThreshImage.at<uchar>(ThreshImage.rows-1, w) > 127) {
        *control_deviation = -100 + w;
        break;
      }
    }
    ROS_INFO("Control deviation set! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Cropped image
    cv::Rect myROI(500, 0, 500, 500);
    cv::Mat croppedImage = image(myROI);
    cv::imshow("cropped", croppedImage);
    ROS_INFO("Cropped! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Lowpass-filtered image
    cv::Mat blurredImage;
    cv::medianBlur( croppedImage, blurredImage, 3 );
    cv::imshow("blurred", blurredImage);
    ROS_INFO("Blurred! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Downscaled image
    cv::Mat downscaledImage;
    cv::resize(blurredImage, downscaledImage, Size(), 0.5, 0.5, cv::INTER_LINEAR);
    cv::imshow("downscaled", downscaledImage);
    ROS_INFO("Downscaled! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // IPM-Transformed image
    cv::Mat transformedImage;

    //TESTING
    // The 4-points at the input image	
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, 250) );
    origPoints.push_back( Point2f(250, 250) );
    origPoints.push_back( Point2f(250, 0) );
    origPoints.push_back( Point2f(0, 0) );

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(50, 250) );
    dstPoints.push_back( Point2f(150, 250) );
    dstPoints.push_back( Point2f(250, 0) );
    dstPoints.push_back( Point2f(0, 0) );
      
    // IPM object
    IPM ipm( Size(250, 250), Size(250, 250), origPoints, dstPoints );

    // transform
		ipm.applyHomography( downscaledImage, transformedImage );		
    
    cv::imshow("transformed", transformedImage);    
    ROS_INFO("Transformed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "control_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  int control_deviation = 0;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control_deviation));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("Hello world!");
  cv::namedWindow("greenfilter");
  cv::namedWindow("cropped");
  cv::namedWindow("blurred");
  cv::namedWindow("downscaled");
  cv::namedWindow("transformed");
  cv::startWindowThread();

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    // simple bang-bang...
    if (control_deviation > 5)
    {
      steering.data = -750;
    }
    else if (control_deviation < -5)
    {
      steering.data = 750;
    }
    else
    {
      steering.data = 0;
    }

    // publish command messages on their topics
    steeringCtrl.publish(steering);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
