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
    ROS_INFO("Received new image! Sizes = %d %d", image.cols, image.rows);
    cv::imshow("raw", image);

    int width = 960, height = 540;

    // Cropped image
    cv::Rect myROI(0, height/4, width, height/4*3);
    cv::Mat croppedImage = HSVImage(myROI);
    cv::imshow("cropped", croppedImage);
    ROS_INFO("Cropped! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Lowpass-filtered image
    cv::Mat blurredImage;
    cv::medianBlur( croppedImage, blurredImage, 3 );
    cv::imshow("blurred", blurredImage);
    ROS_INFO("Blurred! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);


    //TESTING IPM
    // IPM-Transformed image
    cv::Mat transformedImage;
    height = height/4*3;

    // The 4-points at the input image	
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height) );
    origPoints.push_back( Point2f(width, height) );
    origPoints.push_back( Point2f(width, 0) );
    origPoints.push_back( Point2f(0, 0) );

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(395, height) );
    dstPoints.push_back( Point2f(565, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
      
    // IPM transform
    IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );
		ipm.applyHomography( blurredImage, transformedImage );		
    cv::imshow("transformed", transformedImage);    
    ROS_INFO("Transformed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);


    // TESTING LANE DETECTION
    // Thresholding
    cv::Mat ThreshImage;
    inRange(transformedImage, cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImage);
    cv::imshow("thresholded", ThreshImage);    
    ROS_INFO("Thresholded! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Sliding window k-means averaging search for curved path center detection, then curve fitting
    int WINDOW_SIZE = 20;
    std::vector<int> leftLanePoints;
    std::vector<int> rightLanePoints;
    for (int window = 1; window < 15; window++) {
      // Calculate Column Histograms for first window
      cv::Rect roi(0, height - window * WINDOW_SIZE, width, WINDOW_SIZE);
      cv::Mat windowedImage = ThreshImage(roi);
      cv::Mat histogram;
      cv::reduce(windowedImage, histogram, 0, CV_REDUCE_AVG);

      // Histogram peak detection
      double min=0, max=0;
      Point minLoc, maxLoc;
      minMaxLoc(histogram, &min, &max, &minLoc, &maxLoc);
      cv::circle(transformedImage, Point(maxLoc.x, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 10, cv::Scalar(255,255,255), 1, 8, 0);
      int firstPeakX = maxLoc.x;      
      cv::circle(histogram, maxLoc, 12, cv::Scalar(0), CV_FILLED, 8, 0); // fill in found maximum with black pixels

      minMaxLoc(histogram, &min, &max, &minLoc, &maxLoc);
      cv::circle(transformedImage, Point(maxLoc.x, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 10, cv::Scalar(255,255,255), 1, 8, 0);
      int secondPeakX = maxLoc.x;
      int leftPeakX = firstPeakX < secondPeakX ? firstPeakX : secondPeakX;
      int rightPeakX = firstPeakX < secondPeakX ? secondPeakX : firstPeakX;
      leftLanePoints.push_back(leftPeakX);
      rightLanePoints.push_back(rightPeakX);
    }

    cv::imshow("windowed", transformedImage);    
    ROS_INFO("windowed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
    

    // for now, take histogram peaks to calculate control deviation directly
    // int targetDeviationX = 500;
    // int currentDeviationX = leftPeakX + (rightPeakX - leftPeakX) * 3 / 4;
    // *control_deviation = targetDeviationX - currentDeviationX;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'?", msg->encoding.c_str());
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
  cv::namedWindow("raw");
  cv::namedWindow("cropped");
  cv::namedWindow("blurred");
  cv::namedWindow("transformed");
  cv::namedWindow("thresholded");
  cv::namedWindow("windowed");
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
