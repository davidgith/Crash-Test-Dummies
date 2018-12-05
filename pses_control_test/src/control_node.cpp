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
#include <algorithm>    // std::max

#include "IPM.h"

using namespace cv;
using namespace std;

Point getCentroid( InputArray Points )
{
    Point Coord;
    Moments mm = moments( Points, false );
    double moment10 = mm.m10;
    double moment01 = mm.m01;
    double moment00 = mm.m00;
    Coord.x = int(moment10 / moment00);
    Coord.y = int(moment01 / moment00);
    return Coord;
}

bool isLeftTurn(std::vector<Point>& leftmostPoints, std::vector<Point>& rightmostPoints) {
  int leftSteps = 0;
  int rightSteps = 0;

  for (int i = 0; i + 1 < rightmostPoints.size(); i++) {
    if (rightmostPoints.at(i+1).x > rightmostPoints.at(i).x) {
      rightSteps++;
    } else {
      leftSteps++;
    }
  }

  for (int i = 0; i + 1 < leftmostPoints.size(); i++) {
    if (leftmostPoints.at(i+1).x > leftmostPoints.at(i).x) {
      rightSteps++;
    } else {
      leftSteps++;
    }
  }

  return leftSteps > rightSteps;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control)
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
    origPoints.push_back(Point2f(0, height));
    origPoints.push_back(Point2f(width, height));
    origPoints.push_back(Point2f(width, 0));
    origPoints.push_back(Point2f(0, 0));

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(395, height));
    dstPoints.push_back(Point2f(565, height));
    dstPoints.push_back(Point2f(width, 0));
    dstPoints.push_back(Point2f(0, 0));
      
    // IPM transform
    IPM ipm(Size(width, height), Size(width, height), origPoints, dstPoints);
		ipm.applyHomography(blurredImage, transformedImage);		
    cv::imshow("transformed", transformedImage);    
    ROS_INFO("Transformed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);


    // TESTING LANE DETECTION
    // Thresholding
    cv::Mat ThreshImage;
    inRange(transformedImage, cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImage);
    cv::imshow("thresholded", ThreshImage);    
    ROS_INFO("Thresholded! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Sliding window histogram peak search -> clustering around peak
    int WINDOW_SIZE = 10;
    std::vector<Point> firstPoints;
    std::vector<Point> secondPoints;
    for (int window = 1; window <= height/WINDOW_SIZE; window++) {
      // Calculate Column Histograms for first window
      cv::Rect roi(0, height - window * WINDOW_SIZE, width, WINDOW_SIZE);
      cv::Mat windowedImage = ThreshImage(roi);
      cv::Mat histogram;
      cv::reduce(windowedImage, histogram, 0, CV_REDUCE_AVG);

      // Histogram peak detection
      double min=0, max=0;
      Point minLoc, maxLoc;
      // detect window histogram maximum
      minMaxLoc(histogram, &min, &max, &minLoc, &maxLoc);
      int firstPeakX = maxLoc.x;      
      // draw over maximum with black pixels (prevents peak detection around it), then find next maximum
      cv::circle(histogram, maxLoc, 30, cv::Scalar(0), CV_FILLED, 8, 0); 
      minMaxLoc(histogram, &min, &max, &minLoc, &maxLoc);
      int secondPeakX = maxLoc.x;
      // order peaks
      int leftPeakX = firstPeakX < secondPeakX ? firstPeakX : secondPeakX;
      int rightPeakX = firstPeakX < secondPeakX ? secondPeakX : firstPeakX;

      // DEBUG visualization of cluster center
      cv::circle(transformedImage, Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);
      cv::circle(transformedImage, Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);

      // save point
      firstPoints.push_back(Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2));
      secondPoints.push_back(Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2));

      // Obsolete
      /* Buggy small area clustering
      // calculate centroids of cluster pixels and save
      int CLUSTERING_WIDTH = 20;
      cv::Rect leftRoi(std::max(0, leftPeakX - CLUSTERING_WIDTH / 2), height - window * WINDOW_SIZE, std::min(CLUSTERING_WIDTH, width - leftPeakX + CLUSTERING_WIDTH / 2), WINDOW_SIZE);
      cv::Rect rightRoi(std::max(0, rightPeakX - CLUSTERING_WIDTH / 2), height - window * WINDOW_SIZE, std::min(CLUSTERING_WIDTH, width - rightPeakX + CLUSTERING_WIDTH / 2), WINDOW_SIZE);
      Mat leftPixels = ThreshImage(leftRoi);
      Mat rightPixels = ThreshImage(rightRoi);
      Mat leftLanePixels;
      Mat rightLanePixels;
      findNonZero(leftPixels, leftLanePixels);
      findNonZero(rightPixels, rightLanePixels);
      Point leftLanePoint = getCentroid(leftLanePixels);
      Point rightLanePoint = getCentroid(rightLanePixels);

      // correction since subimage does not retain original coordinates
      leftLanePoint.x = leftLanePoint.x + std::max(0, leftPeakX - CLUSTERING_WIDTH / 2); 
      rightLanePoint.x = rightLanePoint.x + std::max(0, rightPeakX - CLUSTERING_WIDTH / 2); 
      leftLanePoint.y = height - window * WINDOW_SIZE + WINDOW_SIZE/2;
      rightLanePoint.y = height - window * WINDOW_SIZE + WINDOW_SIZE/2;

      // save point
      firstPoints.push_back(leftLanePoint);
      secondPoints.push_back(rightLanePoint);

      // DEBUG visualization of detected centroids
      cv::circle(transformedImage, leftLanePoint, 10, cv::Scalar(255,0,0), 1, 8, 0);
      cv::circle(transformedImage, rightLanePoint, 10, cv::Scalar(255,0,0), 1, 8, 0);
      */

      /* Obsolete k-means clustering
      // calculate centroids of cluster pixels and save
      int splitX = leftPeakX + (rightPeakX - leftPeakX) / 2;
      cv::Rect leftRoi(0, height - window * WINDOW_SIZE, splitX, WINDOW_SIZE);
      cv::Rect rightRoi(splitX, height - window * WINDOW_SIZE, width - splitX, WINDOW_SIZE);
      Mat leftPixels = ThreshImage(leftRoi);
      Mat rightPixels = ThreshImage(rightRoi);
      Mat leftLanePixels;
      Mat rightLanePixels;
      findNonZero(leftPixels, leftLanePixels);
      findNonZero(rightPixels, rightLanePixels);
      Point leftLanePoint = getCentroid(leftLanePixels);
      Point rightLanePoint = getCentroid(rightLanePixels);
      rightLanePoint.x = rightLanePoint.x + splitX; //correction since subimage does not retain original coordinates
      leftLanePoint.y = height - window * WINDOW_SIZE + WINDOW_SIZE/2;
      rightLanePoint.y = height - window * WINDOW_SIZE + WINDOW_SIZE/2;
      firstPoints.push_back(leftLanePoint);
      secondPoints.push_back(rightLanePoint);
      */
    }
    ROS_INFO("Sliding window search done! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    std::vector<Point> leftLanePairPoints;
    std::vector<Point> rightLanePairPoints;
    // select interesting point pairs (distance between centroids approximately appropriate)
    for (int i = 0; i < firstPoints.size(); i++) {
      //filter if only one point detected
      if (firstPoints.at(i).x == 0)
        continue;

      //if distance between appropriate values, use these points
      int distanceX = secondPoints.at(i).x - firstPoints.at(i).x;
      if (distanceX < width/2 && distanceX > width/10) {
        leftLanePairPoints.push_back(firstPoints.at(i));
        rightLanePairPoints.push_back(secondPoints.at(i));

        // DEBUG visualization of window of interest
        cv::circle(transformedImage, firstPoints.at(i), 8, cv::Scalar(100,100,0), 1, 8, 0);
        cv::circle(transformedImage, secondPoints.at(i), 8, cv::Scalar(100,100,0), 1, 8, 0);
      }
    }

    std::vector<Point> rightmostPoints;
    // for detected left turn, select rightmost points as right lane
    for (int i = 0; i < firstPoints.size(); i++) {
      //if distance between appropriate values for rightmost point, use rightmost point
      if (secondPoints.at(i).x < width/4*3 && secondPoints.at(i).x > width/4*1) {
        rightmostPoints.push_back(secondPoints.at(i));

        // DEBUG visualization of window of interest
        //cv::circle(transformedImage, secondPoints.at(i), 12, cv::Scalar(200,200,0), 1, 8, 0);
      }
      //else try leftmost point since rightmost point is out of range
      else if (firstPoints.at(i).x < width/4*3 && firstPoints.at(i).x > width/4*1) {
        rightmostPoints.push_back(firstPoints.at(i));

        // DEBUG visualization of window of interest
        //cv::circle(transformedImage, firstPoints.at(i), 12, cv::Scalar(200,200,0), 1, 8, 0);
      } 
    }

    std::vector<Point> leftmostPoints;
    // for detected right turn, select leftmost points as left lane
    for (int i = 0; i < firstPoints.size(); i++) {
      //if distance between appropriate values for leftmost point, use leftmost point
      if (firstPoints.at(i).x < width/4*3 && firstPoints.at(i).x > width/4*1) {
        leftmostPoints.push_back(firstPoints.at(i));

        // DEBUG visualization of window of interest
        //cv::circle(transformedImage, firstPoints.at(i), 14, cv::Scalar(200,200,200), 1, 8, 0);
      } 
      //else try rightmost point since leftmost point is out of range
      else if (secondPoints.at(i).x < width/4*3 && secondPoints.at(i).x > width/4*1) {
        leftmostPoints.push_back(secondPoints.at(i));

        // DEBUG visualization of window of interest
        //cv::circle(transformedImage, secondPoints.at(i), 14, cv::Scalar(200,200,200), 1, 8, 0);
      }
    }
    ROS_INFO("Interesting points selected! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    std::vector<Point> wayPoints;
    // if sufficient two-lane-point-pairs, calculate all pathing points as three-quarters between left and right points
    // else if lane centroid on left half of image, assume right lane visible, else assume left lane visible
    if (leftLanePairPoints.size() > 6) {
      // double lane detected
      for (int i = 0; i < leftLanePairPoints.size(); i++) {
        Point leftLanePoint = leftLanePairPoints.at(i);
        Point rightLanePoint = rightLanePairPoints.at(i);
        Point wayPoint(leftLanePoint.x + (rightLanePoint.x - leftLanePoint.x)*3/4, leftLanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    } else if (isLeftTurn(leftmostPoints, rightmostPoints)) {
      // left turn detected
      for (int i = 0; i < rightmostPoints.size(); i++) {
        Point lanePoint = rightmostPoints.at(i);
        Point wayPoint(std::max(0, lanePoint.x - 60), lanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    } else {
      // right turn detected
      for (int i = 0; i < leftmostPoints.size(); i++) {
        Point lanePoint = leftmostPoints.at(i);
        Point wayPoint(std::min(width, lanePoint.x + 180), lanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    }
    ROS_INFO("Waypoints set! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // DEBUG visualization of waypoints
    for (int i = 0; i < wayPoints.size(); i++) {
      cv::circle(transformedImage, wayPoints.at(i), 10, cv::Scalar(255,0,0), 1, 8, 0);
    }

    // TODO curve fitting a nominal trajectory
    // for now, set steering level such that first waypoint is reached
    if (wayPoints.size() > 0) {
      Point robotLocation(490, 600);
      Point targetLocation = wayPoints.at(0);
      // TODO calculate Lenkwinkel -> calculate steering_lvl
      *control = targetLocation.x - robotLocation.x; 
    }

    // DEBUG visualization
    cv::imshow("windowed", transformedImage);    
    ROS_INFO("Finished planning trajectory! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
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
  int control = 0;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control));

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
    steering.data = control;

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
