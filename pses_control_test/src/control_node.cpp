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
#include <tuple>

#include "QuadProg++.hh"

#include "IPM.h"

using namespace cv;
using namespace std;
using namespace Eigen;

// Model Calibration Macros
#define METER_PER_PIXEL_X 0.0033f
#define METER_PER_PIXEL_Y 0.0048f

#define TARGET_VELOCITY 0.05

#define MODEL_PARAM_L_H 0.1f
#define MODEL_PARAM_L 0.1f

#define MPC_TIMESTEP 0.02
#define MPC_STEPS 50

#define Y_OFFSET 0
#define PHI_K_OFFSET Y_OFFSET + MPC_STEPS
#define U_OFFSET PHI_K_OFFSET + MPC_STEPS

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

// Detect Histogram peak closest to the center of the image or -1 if none found
int findBestHistogramPeakX(cv::Mat& histogram) {
  int curMaxVal = 0, curMaxIndex = -1;
  for (int i = 0; i < histogram.cols; i++) {
    if (histogram.at<uchar>(0,i) < 127)
      continue;

    if (histogram.at<uchar>(0,i) > curMaxVal) {
      curMaxVal = histogram.at<uchar>(0,i);
      curMaxIndex = i;
    } 
    else if (histogram.at<uchar>(0,i) == curMaxVal) {
      if (abs(i - histogram.cols / 2) < abs(curMaxIndex - histogram.cols / 2)) {
        curMaxIndex = i;
      }
    }
  }
  return curMaxIndex;      
}

// Callback on new kinect image
void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control, IPM* ipm)
{
  const bool LEFT_LANE = false;
  const int WINDOW_SIZE = 10;
  const int PEAK_MIN_DISTANCE = 100;
  const int MIN_LANE_POINT_PAIRS = 13;
  const int DETECTION_END_OFFSET_Y = 100;

  clock_t begin = clock();
  
  try
  {
    // Read image
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    int width = image.cols, height = image.rows;
    ROS_INFO("********************");
    ROS_INFO("RECEIVED NEW IMAGE! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Crop image
    cv::Rect myROI(0, height/4, width, height/4*3);
    cv::Mat croppedImage = image(myROI);
    height = height/4*3;
    ROS_INFO("Cropped! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Lowpass-filter image
    cv::Mat blurredImage;
    cv::medianBlur( croppedImage, blurredImage, 3 );
    ROS_INFO("Blurred! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Convert image into HSV color space
    cv::Mat HSVImage;
    cvtColor(blurredImage,HSVImage,CV_BGR2HSV);
    ROS_INFO("HSV! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // IPM-transform image
    cv::Mat transformedImage;
		ipm->applyHomography(HSVImage, transformedImage);		
    ROS_INFO("IPM-transformed! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Threshold the colors to find green and pink
    cv::Mat ThreshImageGreen, ThreshImagePink;
    inRange(transformedImage, cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImageGreen);
    inRange(transformedImage, cv::Scalar(155,50,120),cv::Scalar(175,255,255),ThreshImagePink);
    ROS_INFO("Thresholded! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Sliding window histogram peak search for lane points
    std::vector<std::tuple<Point, Point, Point>> detectedLanePoints;
    for (int window = 1; window <= height/WINDOW_SIZE - DETECTION_END_OFFSET_Y/WINDOW_SIZE; window++) {
      // Calculate column histogram for window
      cv::Rect roi(0, height - window * WINDOW_SIZE, width, WINDOW_SIZE);
      cv::Mat windowedImageGreen = ThreshImageGreen(roi);
      cv::Mat windowedImagePink = ThreshImagePink(roi);
      cv::Mat histogramGreen, histogramPink;
      cv::reduce(windowedImageGreen, histogramGreen, 0, CV_REDUCE_AVG);
      cv::reduce(windowedImagePink, histogramPink, 0, CV_REDUCE_AVG);

      // Find good peaks for green (2)
      int firstPeakX = findBestHistogramPeakX(histogramGreen);
      // Blacken region around found peak
      cv::circle(histogramGreen, Point(firstPeakX, 0), PEAK_MIN_DISTANCE, cv::Scalar(0), CV_FILLED, 8, 0); 
      int secondPeakX = findBestHistogramPeakX(histogramGreen);   
      // Order green peaks
      int leftPeakX = firstPeakX < secondPeakX ? firstPeakX : secondPeakX;
      int rightPeakX = firstPeakX < secondPeakX ? secondPeakX : firstPeakX;

      // Find good peak for pink (1)
      int pinkPeakX = findBestHistogramPeakX(histogramPink);

      // DEBUG visualization of histogram maxima
      cv::circle(transformedImage, Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);
      cv::circle(transformedImage, Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);
      cv::circle(transformedImage, Point(pinkPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2), 3, cv::Scalar(0,255,0), 1, 8, 0);

      // Save points
      auto points = std::make_tuple(Point(leftPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2),
        Point(pinkPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2),
        Point(rightPeakX, height - window * WINDOW_SIZE + WINDOW_SIZE/2));
      detectedLanePoints.push_back(points);
    }
    ROS_INFO("Sliding window search done! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);


    // Select interesting tuples
    std::vector<std::tuple<Point, Point, Point>> usefulPoints;
    for (int i = 0; i < detectedLanePoints.size(); i++) {
      // If middle point exists, this point is interesting
      if (std::get<1>(detectedLanePoints.at(i)).x != -1)
        usefulPoints.push_back(detectedLanePoints.at(i));
      // Otherwise, the other two lane points must exist to be interesting
      else if (std::get<0>(detectedLanePoints.at(i)).x != -1 
        && std::get<2>(detectedLanePoints.at(i)).x != -1)
        usefulPoints.push_back(detectedLanePoints.at(i));
    }
    // Select rightmost points
    std::vector<Point> rightmostPoints;
    for (int i = 0; i < detectedLanePoints.size(); i++) {
      if (std::get<2>(detectedLanePoints.at(i)).x != -1) {
        // If second point detected, use second point
        rightmostPoints.push_back(std::get<2>(detectedLanePoints.at(i)));
      }
      else if (std::get<0>(detectedLanePoints.at(i)).x != -1) {
        // If no second point but first point detected, use first point
        rightmostPoints.push_back(std::get<0>(detectedLanePoints.at(i)));
      }
    }
    // Select leftmost points
    std::vector<Point> leftmostPoints;
    for (int i = 0; i < detectedLanePoints.size(); i++) {
      if (std::get<0>(detectedLanePoints.at(i)).x == -1 && std::get<2>(detectedLanePoints.at(i)).x != -1) {
        // If second point detected and no first point detected, use second point
        leftmostPoints.push_back(std::get<2>(detectedLanePoints.at(i)));
      }
      else if (std::get<0>(detectedLanePoints.at(i)).x != -1 && std::get<2>(detectedLanePoints.at(i)).x != -1) {
        // If second point detected and first point detected, use first point
        leftmostPoints.push_back(std::get<0>(detectedLanePoints.at(i)));
      }
    }
    ROS_INFO("Interesting points selected! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);


    // Generate waypoints from detected lane points
    std::vector<Point> wayPoints;
    if (usefulPoints.size() > MIN_LANE_POINT_PAIRS) {
      // Sufficient useful horizontal tuples -> place waypoints relative to existing tuples
      for (int i = 0; i < usefulPoints.size(); i++) {
        Point leftLanePoint = std::get<0>(usefulPoints.at(i));
        Point middleLanePoint = std::get<1>(usefulPoints.at(i));
        Point rightLanePoint = std::get<2>(usefulPoints.at(i));

        // If the middle point and the point on the driving side exists, place inbetween
        if (LEFT_LANE && middleLanePoint.x != -1 && leftLanePoint.x != -1) {
          Point wayPoint((middleLanePoint.x + leftLanePoint.x) / 2, middleLanePoint.y);
          wayPoints.push_back(wayPoint);
        }
        else if (!LEFT_LANE && middleLanePoint.x != -1 && rightLanePoint.x != -1) {
          Point wayPoint((middleLanePoint.x + rightLanePoint.x) / 2, middleLanePoint.y);
          wayPoints.push_back(wayPoint);
        }
        // Otherwise, if the middle point does not exist, place in-between lane points
        else if (middleLanePoint.x == -1) {
          Point wayPoint(LEFT_LANE ? (leftLanePoint.x * 3 + rightLanePoint.x) / 4 : 
            (leftLanePoint.x + rightLanePoint.x * 3) / 4, middleLanePoint.y);
          wayPoints.push_back(wayPoint);
        }
        // Otherwise, the middle point must exist by construction
        else {
          Point wayPoint(LEFT_LANE ? middleLanePoint.x - 60 : middleLanePoint.x + 60, middleLanePoint.y);
          wayPoints.push_back(wayPoint);
        }
      }
    } 
    else if (isLeftTurn(leftmostPoints, rightmostPoints)) {
      // Left turn fallback -> drive left from rightmost points
      for (int i = 0; i < rightmostPoints.size(); i++) {
        Point lanePoint = rightmostPoints.at(i);
        Point wayPoint(std::max(0, LEFT_LANE ? lanePoint.x - 180 : lanePoint.x - 60), lanePoint.y);
        wayPoints.push_back(wayPoint);
      }
    } 
    else {
      // Right turn fallback -> drive right from leftmost points
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
    if (wayPoints.size() <= 3) {
      ROS_INFO("Too few waypoints, skipping! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
      return;
    }

    // Fit a quadratic polynomial x(y) as nominal trajectory
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> trajectory_coeffs;
    //Optional: add current robot position to fitting waypoints
    xs.push_back(465);
    ys.push_back(height + 60);
    for (int i = 0; i < wayPoints.size(); i++) {
      xs.push_back(wayPoints.at(i).x);
      ys.push_back(wayPoints.at(i).y);
    }
    polyfit(ys, xs, trajectory_coeffs, 2); 
    ROS_INFO("Finished fitting trajectory! t = %f, coeffs: %f %f %f", 
      double(clock() - begin) / CLOCKS_PER_SEC,
      trajectory_coeffs.at(0), trajectory_coeffs.at(1), trajectory_coeffs.at(2));

    // TODO: Calculate MPC solution from trajectory and state
    // Initialize linear discrete system
    quadprogpp::Matrix<double> A_d, B_d, C_d;
    A_d.resize(2,2);
    B_d.resize(2,1);
    C_d.resize(1,2);

    A_d[0][0] = 1;
    A_d[0][1] = TARGET_VELOCITY * MPC_TIMESTEP;
    A_d[1][0] = 0;
    A_d[1][1] = 1;

    B_d[0][0] = ((TARGET_VELOCITY * MPC_TIMESTEP) * (MODEL_PARAM_L_H + (TARGET_VELOCITY * MPC_TIMESTEP) / 2)) / MODEL_PARAM_L;
    B_d[1][0] = (TARGET_VELOCITY * MPC_TIMESTEP) / MODEL_PARAM_L;

    C_d[0][0] = 1;
    C_d[0][1] = 0;

    //solve_quadprog(G, g0, CE, ce0, CI, ci0, x);





    //*control = 50*(targetLocation.x - robotLocation.x); 
    
    ROS_INFO("Finished MPC code! t = %f");

    // DEBUG visualization of original image space + robot space
    cv::Rect helperROI(0, 0, width, 60);
    cv::Mat transformedOriginalImage, transformedFullImage;
    cvtColor(transformedImage,transformedOriginalImage,CV_HSV2BGR);
    cv::Mat tmpMat = transformedOriginalImage(helperROI);
    cv::vconcat(transformedOriginalImage, tmpMat, transformedFullImage);

    // DEBUG Fill in and draw trajectory and robot position
    for (int y = height + 60; y > 0; y--) {
      int x = round(polyeval(trajectory_coeffs,y));
      cv::circle(transformedFullImage, Point(x,y), 1, cv::Scalar(255,0,0), 1, 8, 0);
    }
    cv::circle(transformedFullImage, Point(465, height + 60), 8, cv::Scalar(255,0,0), 4, 8, 0);

    // Show debug visualizations
    cv::imshow("raw", image);
    cv::imshow("transformed", transformedImage);    
    cv::imshow("thresholdedGreen", ThreshImageGreen);  
    cv::imshow("thresholdedPink", ThreshImagePink);    
    cv::imshow("windowed", transformedImage);    
    cv::imshow("windowedOrig", transformedFullImage);    
    ROS_INFO("Finished debug output! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

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

  // Define IPM transformation
  vector<Point2f> origPoints;
  origPoints.push_back(Point2f(0, 405));
  origPoints.push_back(Point2f(960, 405));
  origPoints.push_back(Point2f(960, 0));
  origPoints.push_back(Point2f(0, 0));
  vector<Point2f> dstPoints;
  dstPoints.push_back(Point2f(395, 405));
  dstPoints.push_back(Point2f(565, 405));
  dstPoints.push_back(Point2f(960, 0));
  dstPoints.push_back(Point2f(0, 0));
  IPM ipm(Size(960, 405), Size(960, 405), origPoints, dstPoints);

  // sensor message container
  int control = 0;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control, &ipm));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("Hello world!");
  cv::namedWindow("raw");
  cv::namedWindow("transformed");
  cv::namedWindow("thresholdedGreen");
  cv::namedWindow("thresholdedPink");
  cv::namedWindow("windowed");
  cv::namedWindow("windowedOrig");
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
