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
#include <algorithm>    
#include <eigen3/Eigen/Dense>

#include "QuadProg++.hh"

#include "IPM.h"

using namespace cv;
using namespace std;
using namespace Eigen;

// Model Calibration Macros
#define METER_PER_PIXEL_X 0.0048f
#define METER_PER_PIXEL_Y 0.0033f

/////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  Example code for fitting a polynomial to sample data (using Eigen 3)
 *
 *  Copyright (C) 2014  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2014  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
void polyfit(const std::vector<double> &xv, const std::vector<double> &yv, std::vector<double> &coeff, int order)
{
	Eigen::MatrixXd A(xv.size(), order+1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	Eigen::VectorXd result;

	assert(xv.size() == yv.size());
	assert(xv.size() >= order+1);

	// create matrix
	for (size_t i = 0; i < xv.size(); i++)
	for (size_t j = 0; j < order+1; j++)
		A(i, j) = pow(xv.at(i), j);

	// solve for linear least squares fit
	result = A.householderQr().solve(yv_mapped);

	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}
/////////////////////////////////////////////////////////////////////////////////////////////

// Evaluate a polynomial.
double polyeval(const std::vector<double> coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Detect left or right turn
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

// Callback on new kinect image
void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control)
{
  const bool LEFT_LANE = false;
  const int WINDOW_SIZE = 10;
  const int PEAK_MIN_DISTANCE = 100;
  const int MIN_LANE_POINT_PAIRS = 5;
  const int DETECTION_END_OFFSET_Y = 100;

  clock_t begin = clock();
  
  try
  {
    // Read image
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    int width = image.cols, height = image.rows;
    cv::imshow("raw", image);

    // Convert image into HSV color space
    cv::Mat HSVImage;
    cvtColor(image,HSVImage,CV_BGR2HSV);
    ROS_INFO("Received new image! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Crop image
    cv::Rect myROI(0, height/4, width, height/4*3);
    cv::Mat croppedImage = HSVImage(myROI);
    height = height/4*3;
    //ROS_INFO("Cropped! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Lowpass-filter image
    cv::Mat blurredImage;
    cv::medianBlur( croppedImage, blurredImage, 3 );
    //ROS_INFO("Blurred! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Define IPM transformation
    vector<Point2f> origPoints;
    origPoints.push_back(Point2f(0, height));
    origPoints.push_back(Point2f(width, height));
    origPoints.push_back(Point2f(width, 0));
    origPoints.push_back(Point2f(0, 0));
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(395, height));
    dstPoints.push_back(Point2f(565, height));
    dstPoints.push_back(Point2f(width, 0));
    dstPoints.push_back(Point2f(0, 0));
    IPM ipm(Size(width, height), Size(width, height), origPoints, dstPoints);

    // IPM-transform image
    cv::Mat transformedImage;
		ipm.applyHomography(blurredImage, transformedImage);		
    cv::imshow("transformed", transformedImage);    
    ROS_INFO("IPM-transformed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Threshold the colors to find green
    cv::Mat ThreshImage;
    inRange(transformedImage, cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImage);
    cv::imshow("thresholded", ThreshImage);    
    ROS_INFO("Thresholded! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Sliding window histogram peak search for lane points
    std::vector<Point> firstPoints;
    std::vector<Point> secondPoints;
    for (int window = 1; window <= height/WINDOW_SIZE - DETECTION_END_OFFSET_Y/WINDOW_SIZE; window++) {
      // Calculate column histogram for window
      cv::Rect roi(0, height - window * WINDOW_SIZE, width, WINDOW_SIZE);
      cv::Mat windowedImage = ThreshImage(roi);
      cv::Mat histogram;
      cv::reduce(windowedImage, histogram, 0, CV_REDUCE_AVG);

      // Manual histogram peak detection for one point
      int curMaxVal = 0, curMaxIndex = 0;
      for (int i = 0; i < histogram.cols; i++) {
        if (histogram.at<uchar>(0,i) < 127)
          continue;

        if (histogram.at<uchar>(0,i) > curMaxVal) {
          curMaxVal = histogram.at<uchar>(0,i);
          curMaxIndex = i;
        } 
        else if (histogram.at<uchar>(0,i) == curMaxVal) {
          if (abs(i-histogram.cols/2) < abs(curMaxIndex-histogram.cols/2)) {
            curMaxIndex = i;
          }
        }
      }
      int firstPeakX = curMaxIndex;      


      // Blacken region around found peak
      if (firstPeakX > 0)
        cv::circle(histogram, Point(firstPeakX, 0), PEAK_MIN_DISTANCE, cv::Scalar(0), CV_FILLED, 8, 0); 
      
      // Manual histogram peak detection for second point
      curMaxVal = 0;
      curMaxIndex = 0;
      for (int i = 0; i < histogram.cols; i++) {
        if (histogram.at<uchar>(0,i) < 127)
          continue;

        if (histogram.at<uchar>(0,i) > curMaxVal) {
          curMaxVal = histogram.at<uchar>(0,i);
          curMaxIndex = i;
        } 
        else if (histogram.at<uchar>(0,i) == curMaxVal) {
          if (abs(i-histogram.cols/2) < abs(curMaxIndex-histogram.cols/2)) {
            curMaxIndex = i;
          }
        }
      }
      int secondPeakX = curMaxIndex;     

      // Order peaks
      int leftPeakX = firstPeakX < secondPeakX ? firstPeakX : secondPeakX;
      int rightPeakX = firstPeakX < secondPeakX ? secondPeakX : firstPeakX;

      // DEBUG visualization of histogram maximum
      cv::circle(transformedImage, Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);
      cv::circle(transformedImage, Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);

      // Save points
      firstPoints.push_back(Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2));
      secondPoints.push_back(Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2));
    }
    ROS_INFO("Sliding window search done! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Select interesting points (horizontal pairs)
    std::vector<Point> leftLanePairPoints;
    std::vector<Point> rightLanePairPoints;
    for (int i = 0; i < firstPoints.size(); i++) {
      if (firstPoints.at(i).x == 0) {
        // If only one point detected, skip this pair of points
        continue;
      }

      leftLanePairPoints.push_back(firstPoints.at(i));
      rightLanePairPoints.push_back(secondPoints.at(i));
    }

    // Select interesting points (rightmost points)
    std::vector<Point> rightmostPoints;
    for (int i = 0; i < firstPoints.size(); i++) {
      if (secondPoints.at(i).x != 0) {
        // If second point detected, use second point
        rightmostPoints.push_back(secondPoints.at(i));
      }
      else if (firstPoints.at(i).x != 0) {
        // If no second point but first point detected, use first point
        rightmostPoints.push_back(firstPoints.at(i));
      }
    }

    // Select interesting points (leftmost points)
    std::vector<Point> leftmostPoints;
    for (int i = 0; i < firstPoints.size(); i++) {
      if (firstPoints.at(i).x == 0 && secondPoints.at(i).x != 0) {
        // If second point detected and no first point detected, use second point
        leftmostPoints.push_back(secondPoints.at(i));
      }
      else if (firstPoints.at(i).x != 0 && secondPoints.at(i).x != 0) {
        // If second point detected and first point detected, use first point
        leftmostPoints.push_back(firstPoints.at(i));
      }
    }
    ROS_INFO("Interesting points selected! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Generate waypoints from interesting detected lane points
    std::vector<Point> wayPoints;
    if (leftLanePairPoints.size() > MIN_LANE_POINT_PAIRS) {
      // Sufficient horizontal pairs -> waypoints 3/4 between existing pairs
      for (int i = 0; i < leftLanePairPoints.size(); i++) {
        Point leftLanePoint = leftLanePairPoints.at(i);
        Point rightLanePoint = rightLanePairPoints.at(i);
        Point wayPoint(leftLanePoint.x + (rightLanePoint.x - leftLanePoint.x)*3/4, leftLanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    } 
    else if (isLeftTurn(leftmostPoints, rightmostPoints)) {
      // Left turn -> drive left from rightmost points
      for (int i = 0; i < rightmostPoints.size(); i++) {
        Point lanePoint = rightmostPoints.at(i);
        Point wayPoint(std::max(0, LEFT_LANE ? lanePoint.x - 180 : lanePoint.x - 60), lanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    } 
    else {
      // Right turn -> drive right from leftmost points
      for (int i = 0; i < leftmostPoints.size(); i++) {
        Point lanePoint = leftmostPoints.at(i);
        Point wayPoint(std::min(width, LEFT_LANE ? lanePoint.x + 60: lanePoint.x + 180), lanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    }
    ROS_INFO("Waypoints set! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // DEBUG visualization of waypoints
    for (int i = 0; i < wayPoints.size(); i++) {
      cv::circle(transformedImage, wayPoints.at(i), 10, cv::Scalar(255,0,0), 1, 8, 0);
    }

    // If we could not find more than three wayPoints, we stop and do nothing
    if (wayPoints.size() <= 3) 
      return;

    // Fit a quadratic polynomial x(y) as nominal trajectory
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> trajectory_coeffs;
    for (int i = 0; i < wayPoints.size(); i++) {
      xs.push_back(wayPoints.at(i).x);
      ys.push_back(wayPoints.at(i).y);
    }
    polyfit(ys,xs,trajectory_coeffs,2); 
    ROS_INFO("Finished fitting trajectory! t = %f, coeffs: %f %f %f", 
      double(clock() - begin) / CLOCKS_PER_SEC,
      trajectory_coeffs.at(0), trajectory_coeffs.at(1), trajectory_coeffs.at(2));

    // DEBUG Fill in and draw
    for (int y = 600; y > 0; y--) {
      int x = round(polyeval(trajectory_coeffs,y));
      cv::circle(transformedImage, Point(x,y), 1, cv::Scalar(255,0,0), 1, 8, 0);
    }

    // TODO: Calculate MPC from trajectory
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
    int n, m, p;
    double sum = 0.0;
    char ch;
    
    n = 2;
    G.resize(n, n);
    {
      std::istringstream is("4, -2,"
                            "-2, 4 ");

      for (int i = 0; i < n; i++)	
        for (int j = 0; j < n; j++)
          is >> G[i][j] >> ch;
    }
    
    g0.resize(n);
    {
      std::istringstream is("6.0, 0.0 ");

      for (int i = 0; i < n; i++)
        is >> g0[i] >> ch;
    }
    
    m = 1;
    CE.resize(n, m);
    {
      std::istringstream is("1.0, "
                            "1.0 ");

      for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
          is >> CE[i][j] >> ch;
    } 
    
    ce0.resize(m);
    {
      std::istringstream is("-3.0 ");
      
      for (int j = 0; j < m; j++)
        is >> ce0[j] >> ch;
    }
    
    p = 3;
    CI.resize(n, p);
    {
      std::istringstream is("1.0, 0.0, 1.0, "
                            "0.0, 1.0, 1.0 ");
    
      for (int i = 0; i < n; i++)
        for (int j = 0; j < p; j++)
          is >> CI[i][j] >> ch;
    }
    
    ci0.resize(p);
    {
      std::istringstream is("0.0, 0.0, -2.0 ");

      for (int j = 0; j < p; j++)
        is >> ci0[j] >> ch;
    }
    x.resize(n);

    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);





    //Point robotLocation(490, 600);
    //*control = 50*(targetLocation.x - robotLocation.x); 

    // DEBUG visualization
    //cv::Mat transformedOriginalImage;
    //cvtColor(transformedImage,transformedOriginalImage,CV_HSV2BGR);
    cv::imshow("windowed", transformedImage);    
    ROS_INFO("Finished planning trajectory! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    //cv::waitKey(1);
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
