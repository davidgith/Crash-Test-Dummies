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

// Optimizations
#define DEBUG_VISUALIZATIONS true

// IPM Calibration Configurations
#define ROBOT_POSITION_PIXEL_X 465
#define ROBOT_OFFSET_PIXEL_Y 60
#define METER_PER_PIXEL_X 0.0033f
#define METER_PER_PIXEL_Y 0.0048f

// Waypointing Configurations
#define WINDOW_SIZE 10
#define PEAK_MIN_DISTANCE 100
#define MIN_LANE_POINT_PAIRS 13
#define DETECTION_END_OFFSET_Y 100

// Model Predictive Control Configurations
#define MPC_DT 0.1f
#define MPC_STEPS 25

#define MPC_WEIGHT_U 0.01f
#define MPC_WEIGHT_Y 1000.0f
#define MPC_WEIGHT_PHI 0.0f

#define U_LOWERBOUND (-3.1415f / 4)
#define U_UPPERBOUND (3.1415f / 4)

// Drive Configurations
#define LEFT_LANE false
#define TARGET_VELOCITY 0.2f

// MPC Model Definitions
#define MODEL_PARAM_L_H 0.1f
#define MODEL_PARAM_L 0.1f
#define N_STATES 2
#define N_INPUTS 1
#define N_QUADPROG_VARS (N_INPUTS * MPC_STEPS)

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
    xs.push_back(ROBOT_POSITION_PIXEL_X);
    ys.push_back(height + ROBOT_OFFSET_PIXEL_Y);
    for (int i = 0; i < wayPoints.size(); i++) {
      xs.push_back(wayPoints.at(i).x);
      ys.push_back(wayPoints.at(i).y);
    }
    polyfit(ys, xs, trajectory_coeffs, 2); 
    ROS_INFO("Finished fitting trajectory! t = %f, coeffs: %f %f %f", 
      double(clock() - begin) / CLOCKS_PER_SEC,
      trajectory_coeffs.at(0), trajectory_coeffs.at(1), trajectory_coeffs.at(2));

    // Calculate MPC solution from trajectory and state
    // Initialize linear discrete system
    quadprogpp::Matrix<double> A_d, B_d, C_d;
    A_d.resize(2,2);
    B_d.resize(2,1);
    C_d.resize(1,2);

    A_d[0][0] = 1;
    A_d[0][1] = TARGET_VELOCITY * MPC_DT;
    A_d[1][0] = 0;
    A_d[1][1] = 1;

    B_d[0][0] = ((TARGET_VELOCITY * MPC_DT) * (MODEL_PARAM_L_H + (TARGET_VELOCITY * MPC_DT) / 2)) / MODEL_PARAM_L;
    B_d[1][0] = (TARGET_VELOCITY * MPC_DT) / MODEL_PARAM_L;

    C_d[0][0] = 1;
    C_d[0][1] = 0;

    // Initialize problem weights
    quadprogpp::Matrix<double> Q;
    double P = MPC_WEIGHT_U;
    Q.resize(0, N_STATES, N_STATES);
    Q[0][0] = MPC_WEIGHT_Y;
    Q[0][1] = 0;
    Q[1][0] = 0;
    Q[1][1] = MPC_WEIGHT_PHI;

    // Initialize reference trajectory via kinematic model
    quadprogpp::Matrix<double> x_ref;
    x_ref.resize(N_STATES * MPC_STEPS, 1);
    double y_pixel = height + ROBOT_OFFSET_PIXEL_Y;
    for (int i = 0; i < MPC_STEPS; i++) {
      y_pixel -= TARGET_VELOCITY * MPC_DT / METER_PER_PIXEL_Y;
      double x_ref_pixel = polyeval(trajectory_coeffs, y_pixel);
      double x_ref_meters = (x_ref_pixel - ROBOT_POSITION_PIXEL_X) * METER_PER_PIXEL_X;
      x_ref[2*i][0] = x_ref_meters;
      x_ref[2*i+1][0] = 0; // unneeded since unweighted
    }

    // Initialize quadratic programming problem
    quadprogpp::Matrix<double> B_dash;
    quadprogpp::Vector<double> tmpVec1;
    quadprogpp::Vector<double> tmpVec2;
    B_dash.resize(0, N_STATES * MPC_STEPS, N_INPUTS * MPC_STEPS);
    tmpVec1.resize(0, N_INPUTS * MPC_STEPS);
    tmpVec2.resize(0, N_INPUTS * MPC_STEPS);
    tmpVec1[0] = B_d[0][0];
    tmpVec2[0] = B_d[1][0];
    for (int row = 0; row < MPC_STEPS; row++) {
      // calculate 2 rows of B_dash each iteration
      for (int col = tmpVec1.size()-1; col > 0; col--) {
        tmpVec1[col] = tmpVec1[col-1];
        tmpVec2[col] = tmpVec2[col-1];
      }
      tmpVec1[0] = A_d[0][0] * tmpVec1[1] + A_d[0][1] * tmpVec2[1];
      tmpVec2[0] = A_d[1][0] * tmpVec1[1] + A_d[1][1] * tmpVec2[1];
    
      for (int col = 0; col < tmpVec1.size(); col++) {
        B_dash[2*row][col] = tmpVec1[col];
        B_dash[2*row+1][col] = tmpVec2[col];
      }
    }
      
    quadprogpp::Matrix<double> Q_dash;
    Q_dash.resize(0, N_STATES * MPC_STEPS, N_STATES * MPC_STEPS);
    for (int row = 0; row < MPC_STEPS; row++) {
      Q_dash[2*row][2*row] = Q[0][0];
      Q_dash[2*row][2*row+1] = Q[0][1];
      Q_dash[2*row+1][2*row] = Q[1][0];
      Q_dash[2*row+1][2*row+1] = Q[1][1];
    }

    quadprogpp::Matrix<double> P_dash;
    P_dash.resize(0, N_INPUTS * MPC_STEPS, N_INPUTS * MPC_STEPS);
    for (int row = 0; row < MPC_STEPS; row++) {
      P_dash[row][row] = P;
    }

    // TODO get better base state and use predicted trajectory with delay
    quadprogpp::Matrix<double> x0;
    x0.resize(0, N_STATES, 1);
    x0[0][0] = 0;
    x0[1][0] = 0;

    quadprogpp::Matrix<double> A_dash;
    A_dash.resize(N_STATES * MPC_STEPS, N_STATES);
    quadprogpp::Matrix<double> _tmpMat = A_d;
    for (int i = 0; i < MPC_STEPS; i++) {
      A_dash[2*i][0] = _tmpMat[0][0];
      A_dash[2*i][1] = _tmpMat[0][1];
      A_dash[2*i+1][0] = _tmpMat[1][0];
      A_dash[2*i+1][1] = _tmpMat[1][1];
      _tmpMat = _tmpMat % A_d;
    }

    quadprogpp::Matrix<double> chi0 = A_dash % x0 - x_ref;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    G = transpose(B_dash) % Q_dash % B_dash + P_dash;
    quadprogpp::Matrix<double> g0Mat = transpose(chi0) % Q_dash % B_dash;
    g0 = g0Mat.extractRow(0);

    // ROS_INFO("Dim Info! B_dash %d %d, Q_dash %d %d, P_dash %d %d, chi0 %d %d, G %d %d, g0Mat %d %d", 
    //   B_dash.nrows(), B_dash.ncols(), 
    //   Q_dash.nrows(), Q_dash.ncols(), 
    //   P_dash.nrows(), P_dash.ncols(), 
    //   chi0.nrows(), chi0.ncols(), 
    //   G.nrows(), G.ncols(), 
    //   g0Mat.nrows(), g0Mat.ncols());
    // for (int i = 0; i < MPC_STEPS; i++)
    //   ROS_INFO("g0(%d) = %f, B_dash = %f, chi0 = %f", i, g0[i], B_dash[i][0], chi0[i][0]);

    // inequality constraints on input (10, 11) 
    CI.resize(0, N_QUADPROG_VARS * 2, N_QUADPROG_VARS);
    for (int i = 0; i < N_QUADPROG_VARS; i++) {
      CI[i][i] = 1; //equation 10
      CI[N_QUADPROG_VARS+i][i] = -1; //equation 11
    }
    CI = transpose(CI);
    ci0.resize(0, N_QUADPROG_VARS * 2);
    for (int i = 0; i < N_QUADPROG_VARS; i++) {
      ci0[i] = -U_LOWERBOUND;
      ci0[N_QUADPROG_VARS+i] = U_UPPERBOUND;
    }

    // equality constraint on state e.g. (12, 13, 16, 17) are skipped
    CE.resize(0, N_QUADPROG_VARS, 0);
    ce0.resize(0, 0);
    ROS_INFO("Finished setting up QP problem! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Solve control input via quadratic programming solver
    quadprogpp::Vector<double> u;
    u.resize(N_QUADPROG_VARS);
    auto test = solve_quadprog(G, g0, CE, ce0, CI, ci0, u);
    
    for (int i = 0; i < N_QUADPROG_VARS; i++)
      ROS_INFO("Finished MPC test! u(%f) = %f", MPC_DT * i, u[i]);

    //for now, just use traditional MPC (first control input)
    *control = round(750 * u[0] / U_UPPERBOUND); // TODO kennlinie einsetzen oder kaskadenregelung

    ROS_INFO("Finished MPC code! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

    // Unnecessary visualizations
    if (DEBUG_VISUALIZATIONS) {
      // Visualization of waypoints
      for (int i = 0; i < wayPoints.size(); i++) {
        cv::circle(transformedImage, wayPoints.at(i), 10, cv::Scalar(255,0,0), 1, 8, 0);
      }

      // Visualization of original image space + robot space
      cv::Rect helperROI(0, 0, width, 60);
      cv::Mat transformedOriginalImage, transformedFullImage;
      cvtColor(transformedImage, transformedOriginalImage, CV_HSV2BGR);
      cv::Mat tmpMat = transformedOriginalImage(helperROI);
      cv::vconcat(transformedOriginalImage, tmpMat, transformedFullImage);

      // Fill in and draw trajectory and robot position
      for (int y = height + ROBOT_OFFSET_PIXEL_Y; y > 0; y--) {
        int x = round(polyeval(trajectory_coeffs, y));
        cv::circle(transformedFullImage, Point(x, y), 1, cv::Scalar(255,0,0), 1, 8, 0);
      }
      cv::circle(transformedFullImage, Point(ROBOT_POSITION_PIXEL_X, height + ROBOT_OFFSET_PIXEL_Y), 8, cv::Scalar(255,0,0), 4, 8, 0);

      // Draw predicted MPC trajectory
      quadprogpp::Matrix<double> predicted_state = x0;
      double predicted_y = height + ROBOT_OFFSET_PIXEL_Y;
      for (int i = 0; i < MPC_STEPS; i++) {
        predicted_state = A_d % predicted_state + B_d * u[i];
        predicted_y -= TARGET_VELOCITY * MPC_DT / METER_PER_PIXEL_Y;
        double predicted_x = predicted_state[0][0] / METER_PER_PIXEL_X + ROBOT_POSITION_PIXEL_X;
        cv::circle(transformedFullImage, Point(round(predicted_x), round(predicted_y)), 2, cv::Scalar(0,0,255), -1, 8, 0);
      }

      // Show debug pictures
      cv::imshow("raw", image);
      cv::imshow("transformed", transformedImage);    
      cv::imshow("thresholdedGreen", ThreshImageGreen);  
      cv::imshow("thresholdedPink", ThreshImagePink);    
      cv::imshow("windowed", transformedImage);    
      cv::imshow("windowedOrig", transformedFullImage);    
      ROS_INFO("Finished debug output! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
    }

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

  cv::namedWindow("raw");
  cv::namedWindow("transformed");
  cv::namedWindow("thresholdedGreen");
  cv::namedWindow("thresholdedPink");
  cv::namedWindow("windowed");
  cv::namedWindow("windowedOrig");
  cv::startWindowThread();

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(100);
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
