/**
 * @file "MPCController.cpp"
 * @brief MPCController class implementation files, containing the main controller.
 *
*/

#include "MPCController.h"

#include <eigen3/Eigen/Dense>
#include <bits/types/clock_t.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core_c.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <pses_control_test/ParamsConfig.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <sys/time.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <tuple>
#include <vector>

#include "Array.hh"
#include "IPM.h"
#include "PolyUtils.h"
#include "QuadProg++.hh"

using namespace cv;
using namespace std;
using namespace Eigen;

// IPM Calibration Configurations
#define ROBOT_POSITION_PIXEL_X 453
#define ROBOT_OFFSET_PIXEL_Y 76
#define METER_PER_PIXEL_X 0.0038f
#define METER_PER_PIXEL_Y 0.005f

// Waypointing Configurations
#define WINDOW_SIZE 10
#define PEAK_MIN_DISTANCE 100
#define MIN_LANE_POINT_PAIRS 3
#define LANE_WIDTH_PIXELS 240

// Model Predictive Control Parameters
#define MPC_WEIGHT_PHI 0.0f
#define U_LOWERBOUND (-3.1415f * 21 / 180)
#define U_UPPERBOUND (3.1415f * 21 / 180)
#define MAX_DELTA_U 0.1f

// MPC Model Definitions
#define MODEL_PARAM_L_H 0.13f
#define MODEL_PARAM_L 0.26f
#define N_STATES 2
#define N_INPUTS 1

#define STOP_SIGNAL -1024

#define STOP_SIGN_COUNTDOWN 2.5f
#define STOP_TIME 3
#define SIGN_COOLDOWN 10

namespace mpc {
	MPCController::MPCController() {
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
		ipm = new IPM(Size(960, 405), Size(960, 405), origPoints, dstPoints);
	}

	MPCController::~MPCController() {
		delete ipm;
	}

	bool MPCController::update(float deltaTime) {
		// Update driving lane target
		drivingLane = drivingLane + laneChangeRate * (targetDrivingLane - drivingLane);

		// Update time and check if new control step is triggered
		// Assumption: deltaTime < currTImeSinceInput
		currTimeSinceInput += deltaTime;
		if (currTimeSinceInput > mpcTimestep) {
			currTimeSinceInput -= mpcTimestep;

			// Catch too big leftover numbers
			if (currTimeSinceInput > mpcTimestep)
				currTimeSinceInput = 0;

			return true;
		}
		return false;
	}

	int MPCController::getNextSteeringControl() {
		std::lock_guard<std::mutex> lck (u_mutex);

		if (!u_queue.empty()) {
			double phi_L = u_queue.front();
			u_queue.pop();
			int u_steering = round(1000 * phi_L / U_UPPERBOUND);
			u_steering = max(-1000, min(1000, u_steering));
			return u_steering;
		}

		return 0;
	}

	int MPCController::getNextMotorControl() {
		// If we are stopping right now, wait until the stopping time has passed
		double currTime = ros::Time::now().toSec();
		if (currTime < lastStopTime + STOP_TIME + STOP_SIGN_COUNTDOWN && currTime > lastStopTime + STOP_SIGN_COUNTDOWN) {
			return 0;
		}

		// Continue driving if next control steps exist
		if (!u_queue.empty() && u_queue.front() != STOP_SIGNAL) {
			return targetVelocity != 0 ? 100 + 444 * targetVelocity : 0;
		}

		return 0;
	}

	void MPCController::reconfigureParameters(pses_control_test::ParamsConfig &config, uint32_t level)
	{
	  showDebugOutputs = config.debug_output;

	  mpcUseDeadtimeCompensation = config.mpc_deadtime_comp;
	  mpcTimestep = config.mpc_timestep;
	  mpcNumberTimesteps = config.mpc_number_timesteps;
	  mpcRecalcInterval = config.mpc_update_rate;
	  mpcWeightY = config.mpc_weight_state;
	  mpcWeightInput = config.mpc_weight_ctrl;

	  detectionEndOffsetY = config.detection_end_offset;

	  targetDrivingLane = config.ctrl_lane;
	  targetVelocity = config.ctrl_velocity;

	  useDirectTrajectory = config.direct_trajectory;
	  directTrajectoryDiscount = config.direct_trajectory_discount;
	  laneChangeRate = config.lane_change_rate;
	}

	// Detect left or right turn
	bool MPCController::isLeftTurn(std::vector<Point>& leftmostPoints, std::vector<Point>& rightmostPoints) {
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
	int MPCController::findBestHistogramPeakX(cv::Mat& histogram) {
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

	int MPCController::getWaypointXFromRightLanePoint(const Point& lanePoint) {
		return lanePoint.x - round((3 - drivingLane) * LANE_WIDTH_PIXELS/4);
	}

	int MPCController::getWaypointXFromLeftLanePoint(const Point& lanePoint) {
		return lanePoint.x + round((1 + drivingLane) * LANE_WIDTH_PIXELS/4);
	}

	int MPCController::getWaypointXFromMiddleLanePoint(const Point& lanePoint) {
		return lanePoint.x + round((-1 + drivingLane) * LANE_WIDTH_PIXELS/4);
	}

	int MPCController::getWaypointXFromOuterLanePoints(const Point& leftLanePoint,
			const Point& rightLanePoint) {
		return round((leftLanePoint.x * (3 - drivingLane) + rightLanePoint.x * (1 + drivingLane)) / 4);
	}

	Point MPCController::getWaypointXFromLanePoints(const Point& leftLanePoint,
			const Point& middleLanePoint, const Point& rightLanePoint) {
		// If the middle point and the point on the driving side exists, place inbetween
		if (drivingLane < 1 && middleLanePoint.x != -1 && leftLanePoint.x != -1) {
			return Point(round(middleLanePoint.x * (0.5f + drivingLane/2) + leftLanePoint.x * (0.5f - drivingLane/2)), middleLanePoint.y);
		} else if (drivingLane >= 1 && middleLanePoint.x != -1 && rightLanePoint.x != -1) {
			return Point(round(middleLanePoint.x * (1.5f - drivingLane/2) + rightLanePoint.x * (-0.5f + drivingLane/2)), middleLanePoint.y);
		} else if (middleLanePoint.x == -1) {
			return Point(getWaypointXFromOuterLanePoints(leftLanePoint, rightLanePoint), middleLanePoint.y);
		} else {
			return Point(getWaypointXFromMiddleLanePoint(middleLanePoint), middleLanePoint.y);
		}
	}

	void MPCController::processAndTransformImage(const sensor_msgs::ImageConstPtr& msg,
			cv::Mat& transformedImage) {
		// Read image
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		int width = image.cols, height = image.rows;

		// Crop image
		cv::Rect myROI(0, height / 4, width, height / 4 * 3);
		cv::Mat croppedImage = image(myROI);
		height = height / 4 * 3;

		// Lowpass-filter image
		cv::Mat blurredImage;
		cv::medianBlur(croppedImage, blurredImage, 3);

		// Convert image into HSV color space
		cv::Mat HSVImage;
		cvtColor(blurredImage, HSVImage, CV_BGR2HSV);

		// IPM-transform image
		ipm->applyHomography(HSVImage, transformedImage);
	}

	void MPCController::findLanePoints(const cv::Mat& transformedImage, const cv::Mat& ThreshImageGreen,
			const cv::Mat& ThreshImagePink, std::vector<std::tuple<Point, Point, Point> >& detectedLanePoints) {
		for (int window = 1; window <= transformedImage.rows / WINDOW_SIZE - detectionEndOffsetY / WINDOW_SIZE; window++) {
			// Calculate column histogram for window
			cv::Rect roi(0, transformedImage.rows - window * WINDOW_SIZE, transformedImage.cols, WINDOW_SIZE);
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
			cv::circle(transformedImage, Point(leftPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2), 3,
					cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(transformedImage, Point(rightPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2), 3,
					cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(transformedImage, Point(pinkPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2), 3,
					cv::Scalar(0, 255, 0), 1, 8, 0);

			// Save points
			auto points = std::make_tuple(Point(leftPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2),
					Point(pinkPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2),
					Point(rightPeakX, transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2));
			detectedLanePoints.push_back(points);
		}
	}

	void MPCController::fillPinkLine(const cv::Mat& transformedImage,
			std::vector<std::tuple<Point, Point, Point> >& detectedLanePoints) {
		// Fill in between pink points
		Point lastPink = Point(-1, -1);
		for (int i = detectedLanePoints.size() - 1; i >= 0; i--) {
			Point currPink = std::get<1>(detectedLanePoints.at(i));
			if (currPink.x != -1) {
				if (lastPink.x != -1) {
					cv::line(transformedImage, lastPink, currPink, cv::Scalar(165, 150, 255), 7);
				}
				lastPink = currPink;
			}
		}

		// Continue line backwards
		Point bestPrevLastPink = Point(-1, -1);
		for (int i = detectedLanePoints.size() - 1; i >= 0; i--) {
			Point currPink = std::get<1>(detectedLanePoints.at(i));
			if (currPink.x != -1 && currPink.y != lastPink.y && currPink.y + 80 > lastPink.y) {
				bestPrevLastPink = currPink;
				break;
			}
		}
		if (lastPink.x != -1 && bestPrevLastPink.x != -1) {
			Point delta = lastPink - bestPrevLastPink;
			int deltaStepsUntilOutOfBounds = (transformedImage.rows + ROBOT_OFFSET_PIXEL_Y - lastPink.y - 1) / delta.y;
			//ROS_INFO("test %d %d %d %d %d", deltaStepsUntilOutOfBounds, delta.x, delta.y, bestPrevLastPink.x, bestPrevLastPink.y);
			Point drawEndPoint = Point(lastPink.x + delta.x * deltaStepsUntilOutOfBounds,
					lastPink.y + delta.y * deltaStepsUntilOutOfBounds);
			cv::line(transformedImage, lastPink, drawEndPoint, cv::Scalar(165, 150, 255), 7);
		}
	}

	void MPCController::sortLanePoints(std::vector<std::tuple<Point, Point, Point> > detectedLanePoints,
			std::vector<std::tuple<Point, Point, Point> >& usefulPoints, std::vector<Point>& rightmostPoints,
			std::vector<Point>& leftmostPoints) {
		// Select interesting tuples
		for (int i = 0; i < detectedLanePoints.size(); i++) {
			// If middle point exists, this point is interesting
			if (std::get<1>(detectedLanePoints.at(i)).x != -1)
				usefulPoints.push_back(detectedLanePoints.at(i));
			else
			// Otherwise, the other two lane points must exist to be interesting
			if (std::get<0>(detectedLanePoints.at(i)).x != -1 && std::get<2>(detectedLanePoints.at(i)).x != -1)
				usefulPoints.push_back(detectedLanePoints.at(i));
		}
		// Select rightmost points
		for (int i = 0; i < detectedLanePoints.size(); i++) {
			if (std::get<2>(detectedLanePoints.at(i)).x != -1) {
				// If second point detected, use second point
				rightmostPoints.push_back(std::get<2>(detectedLanePoints.at(i)));
			} else if (std::get<0>(detectedLanePoints.at(i)).x != -1) {
				// If no second point but first point detected, use first point
				rightmostPoints.push_back(std::get<0>(detectedLanePoints.at(i)));
			}
		}
		// Select leftmost points
		for (int i = 0; i < detectedLanePoints.size(); i++) {
			if (std::get<0>(detectedLanePoints.at(i)).x == -1 && std::get<2>(detectedLanePoints.at(i)).x != -1) {
				// If second point detected and no first point detected, use second point
				leftmostPoints.push_back(std::get<2>(detectedLanePoints.at(i)));
			} else if (std::get<0>(detectedLanePoints.at(i)).x != -1 && std::get<2>(detectedLanePoints.at(i)).x != -1) {
				// If second point detected and first point detected, use first point
				leftmostPoints.push_back(std::get<0>(detectedLanePoints.at(i)));
			}
		}
	}

	void MPCController::generateWaypoints(const cv::Mat& transformedImage, std::vector<Point> rightmostPoints,
			std::vector<Point> leftmostPoints, std::vector<std::tuple<Point, Point, Point> > usefulPointTuples,
			std::vector<Point>& wayPoints) {
		if (usefulPointTuples.size() >= MIN_LANE_POINT_PAIRS) {
			std::vector<Point> usefulWaypoints;

			// Sufficient useful horizontal tuples -> place waypoints relative to existing tuples
			for (int i = 0; i < usefulPointTuples.size(); i++) {
				// The useful left/right points are not necessarily on the left/right lane
				Point middleLanePoint = std::get<1>(usefulPointTuples.at(i));
				Point leftLanePoint(-1, -1);
				Point rightLanePoint(-1, -1);

				if (std::get<0>(usefulPointTuples.at(i)).x == -1 && std::get<2>(usefulPointTuples.at(i)).x != -1) {
					// If there is only one sidelane point, it is placed in relation to the existing middle point
					if (std::get<2>(usefulPointTuples.at(i)).x > middleLanePoint.x)
						rightLanePoint = std::get<2>(usefulPointTuples.at(i));
					else
						leftLanePoint = std::get<2>(usefulPointTuples.at(i));
				} else {
					// Else take the detected points
					leftLanePoint = std::get<0>(usefulPointTuples.at(i));
					rightLanePoint = std::get<2>(usefulPointTuples.at(i));
				}

				Point wayPoint = getWaypointXFromLanePoints(leftLanePoint, middleLanePoint, rightLanePoint);
				usefulWaypoints.push_back(wayPoint);
			}

			// Waypoint post-processing: Infer more waypoints from left/right points
			int leftmostIndex = 0;
			int rightmostIndex = 0;
			int usefulIndex = 0;

			// Get average x of useful waypoints
			int avgWaypointX = 0;
			for (int i = 0; i < usefulWaypoints.size(); i++)
				avgWaypointX += usefulWaypoints.at(i).x;
			avgWaypointX /= usefulWaypoints.size();

			// Now build the actual waypoints:
			for (int window = 1; window <= transformedImage.rows / WINDOW_SIZE - detectionEndOffsetY / WINDOW_SIZE; window++) {
				// The current point height of this window is given by the following
				int currHeight = transformedImage.rows - window * WINDOW_SIZE + WINDOW_SIZE / 2;

				// Cycle through points until at the correct height
				while (usefulIndex < usefulWaypoints.size() && usefulWaypoints.at(usefulIndex).y > currHeight) {
					usefulIndex++;
				}
				while (rightmostIndex < rightmostPoints.size() && rightmostPoints.at(rightmostIndex).y > currHeight) {
					rightmostIndex++;
				}

				// If a useful point exists for this window, use the useful point and increment pointer
				if (usefulIndex < usefulWaypoints.size() && usefulWaypoints.at(usefulIndex).y == currHeight) {
					wayPoints.push_back(usefulWaypoints.at(usefulIndex));
				} else

				// Otherwise, the rightmost point could be used to infer from
				if (rightmostIndex < rightmostPoints.size() && rightmostPoints.at(rightmostIndex).y == currHeight) {
					// Treat it as the right lane point if it is right of the average x of the useful waypoints
					if (rightmostPoints.at(rightmostIndex).x > avgWaypointX + 40) {
						Point lanePoint = rightmostPoints.at(rightmostIndex);
						Point wayPoint(std::max(0, getWaypointXFromRightLanePoint(lanePoint)), lanePoint.y);
						wayPoints.push_back(wayPoint);
					} else
					// Otherwise it is the left lane point
					if (rightmostPoints.at(rightmostIndex).x < avgWaypointX - 40) {
						Point lanePoint = rightmostPoints.at(rightmostIndex);
						Point wayPoint(std::min(transformedImage.cols, getWaypointXFromLeftLanePoint(lanePoint)), lanePoint.y);
						wayPoints.push_back(wayPoint);
					}
				}
			}
		} else if (isLeftTurn(leftmostPoints, rightmostPoints)) {
			// If we do not have enough useful point pairs, we fallback to detecting leftward/rightward
			// Left turn fallback -> drive left from rightmost points
			for (int i = 0; i < rightmostPoints.size(); i++) {
				Point lanePoint = rightmostPoints.at(i);
				Point wayPoint(std::max(0, getWaypointXFromRightLanePoint(lanePoint)), lanePoint.y);
				wayPoints.push_back(wayPoint);
			}
		} else {
			// Right turn fallback -> drive right from leftmost points
			for (int i = 0; i < leftmostPoints.size(); i++) {
				Point lanePoint = leftmostPoints.at(i);
				Point wayPoint(std::min(transformedImage.cols, getWaypointXFromLeftLanePoint(lanePoint)), lanePoint.y);
				wayPoints.push_back(wayPoint);
			}
		}
	}

	void MPCController::fitTrajectoryToWaypoints(const cv::Mat& transformedImage, std::vector<Point>& wayPoints,
			std::vector<double>& trajectory_coeffs) {
		// Fit a polynomial x(y) as nominal trajectory
		std::vector<double> xs;
		std::vector<double> ys;

		if (useDirectTrajectory) {
			// Linear polynomial from robot position to waypoint
			xs.push_back(ROBOT_POSITION_PIXEL_X);
			ys.push_back(transformedImage.rows + ROBOT_OFFSET_PIXEL_Y);
			xs.push_back((1 - directTrajectoryDiscount) * wayPoints.at(0).x + directTrajectoryDiscount * ROBOT_POSITION_PIXEL_X);
			ys.push_back(wayPoints.at(0).y);
			polyfit(ys, xs, trajectory_coeffs, 1);
		} else {
			// Quadratic polynomial:
			if (INTERPOLATE_WITH_CURRENT_POSITION) {
				xs.push_back(ROBOT_POSITION_PIXEL_X);
				ys.push_back(transformedImage.rows + ROBOT_OFFSET_PIXEL_Y);
			}
			for (int i = 0; i < wayPoints.size(); i++) {
				xs.push_back(wayPoints.at(i).x);
				ys.push_back(wayPoints.at(i).y);
			}
			polyfit(ys, xs, trajectory_coeffs, 2);
		}
	}

	void MPCController::solveMPCProblem(const cv::Mat& transformedImage, const std::vector<double>& trajectory_coeffs, clock_t begin,
			std::queue<double> prev_u_queue, const cv::Mat& transformedFullImage, quadprogpp::Vector<double>& optimal_u) {
		int N_QUADPROG_VARS = (N_INPUTS * mpcNumberTimesteps);

		// Calculate MPC solution from trajectory and state
		// Initialize linear discrete system
		quadprogpp::Matrix<double> A_d, B_d, C_d;
		A_d.resize(2, 2);
		B_d.resize(2, 1);
		C_d.resize(1, 2);

		A_d[0][0] = 1;
		A_d[0][1] = targetVelocity * mpcTimestep;
		A_d[1][0] = 0;
		A_d[1][1] = 1;
		B_d[0][0] = ((targetVelocity * mpcTimestep) * (MODEL_PARAM_L_H + (targetVelocity * mpcTimestep) / 2)) / MODEL_PARAM_L;
		B_d[1][0] = (targetVelocity * mpcTimestep) / MODEL_PARAM_L;
		C_d[0][0] = 1;
		C_d[0][1] = 0;

		// Initialize problem weights
		quadprogpp::Matrix<double> Q;
		double P = mpcWeightInput;
		Q.resize(0, N_STATES, N_STATES);

		Q[0][0] = mpcWeightY;
		Q[0][1] = 0;
		Q[1][0] = 0;
		Q[1][1] = MPC_WEIGHT_PHI;

		// Initialize reference trajectory via kinematic model
		quadprogpp::Matrix<double> x_ref;
		x_ref.resize(N_STATES * mpcNumberTimesteps, 1);
		double y_pixel = transformedImage.rows + ROBOT_OFFSET_PIXEL_Y;
		for (int i = 0; i < mpcNumberTimesteps; i++) {
			y_pixel -= targetVelocity * mpcTimestep / METER_PER_PIXEL_Y;
			double x_ref_pixel = polyeval(trajectory_coeffs, y_pixel);
			double x_ref_meters = (x_ref_pixel - ROBOT_POSITION_PIXEL_X) * METER_PER_PIXEL_X;
			x_ref[2 * i][0] = x_ref_meters;
			x_ref[2 * i + 1][0] = 0; // unneeded since unweighted
		}

		// Initialize quadratic programming problem
		quadprogpp::Matrix<double> B_dash;
		quadprogpp::Vector<double> oddRow;
		quadprogpp::Vector<double> evenRow;
		B_dash.resize(0, N_STATES * mpcNumberTimesteps, N_INPUTS * mpcNumberTimesteps);
		oddRow.resize(0, N_INPUTS * mpcNumberTimesteps);
		evenRow.resize(0, N_INPUTS * mpcNumberTimesteps);

		oddRow[0] = B_d[0][0];
		evenRow[0] = B_d[1][0];
		for (int row = 0; row < mpcNumberTimesteps; row++) {
			// calculate 2 rows of B_dash each iteration
			for (int col = oddRow.size() - 1; col > 0; col--) {
				oddRow[col] = oddRow[col - 1];
				evenRow[col] = evenRow[col - 1];
			}
			oddRow[0] = A_d[0][0] * oddRow[1] + A_d[0][1] * evenRow[1];
			evenRow[0] = A_d[1][0] * oddRow[1] + A_d[1][1] * evenRow[1];
			for (int col = 0; col < oddRow.size(); col++) {
				B_dash[2 * row][col] = oddRow[col];
				B_dash[2 * row + 1][col] = evenRow[col];
			}
		}

		quadprogpp::Matrix<double> Q_dash;
		Q_dash.resize(0, N_STATES * mpcNumberTimesteps, N_STATES * mpcNumberTimesteps);
		for (int row = 0; row < mpcNumberTimesteps; row++) {
			Q_dash[2 * row][2 * row] = Q[0][0];
			Q_dash[2 * row][2 * row + 1] = Q[0][1];
			Q_dash[2 * row + 1][2 * row] = Q[1][0];
			Q_dash[2 * row + 1][2 * row + 1] = Q[1][1];
		}

		quadprogpp::Matrix<double> P_dash;
		P_dash.resize(0, N_INPUTS * mpcNumberTimesteps, N_INPUTS * mpcNumberTimesteps);
		for (int row = 0; row < mpcNumberTimesteps; row++) {
			P_dash[row][row] = P;
		}

		// Get initial state
		quadprogpp::Matrix<double> x0;
		x0.resize(0, N_STATES, 1);
		x0[0][0] = 0;
		x0[1][0] = 0;

		// Perform deadtime compensation
		int compensation_steps = round(double(clock() - begin) / CLOCKS_PER_SEC / mpcTimestep);
		if (mpcUseDeadtimeCompensation) {
			for (int i = 0; i < compensation_steps; i++) {
				if (prev_u_queue.empty())
					break;
				x0 = A_d * x0 + B_d * prev_u_queue.front();
				prev_u_queue.pop();
			}
		}

		// Calculate variables for quadratic program
		quadprogpp::Matrix<double> A_dash;
		A_dash.resize(N_STATES * mpcNumberTimesteps, N_STATES);
		quadprogpp::Matrix<double> _tmpMat = A_d;
		for (int i = 0; i < mpcNumberTimesteps; i++) {
			A_dash[2 * i][0] = _tmpMat[0][0];
			A_dash[2 * i][1] = _tmpMat[0][1];
			A_dash[2 * i + 1][0] = _tmpMat[1][0];
			A_dash[2 * i + 1][1] = _tmpMat[1][1];
			_tmpMat = _tmpMat * A_d;
		}

		quadprogpp::Matrix<double> chi0 = A_dash * x0 - x_ref;
		quadprogpp::Matrix<double> G = transpose(B_dash) * Q_dash * B_dash + P_dash;
		quadprogpp::Vector<double> g0 = (transpose(chi0) * Q_dash * B_dash).extractRow(0);
		quadprogpp::Matrix<double> CE, CI;
		CE.resize(0, N_QUADPROG_VARS, 0);
		CI.resize(0, N_QUADPROG_VARS * 4, N_QUADPROG_VARS);
		quadprogpp::Vector<double> ce0, ci0, x;
		ce0.resize(0, 0);
		ci0.resize(0, N_QUADPROG_VARS * 4);

		// inequality constraints on input (10, 11)
		for (int i = 0; i < N_QUADPROG_VARS; i++) {
			CI[i][i] = 1; //equation 10
			CI[N_QUADPROG_VARS + i][i] = -1; //equation 11
		}
		for (int i = 0; i < N_QUADPROG_VARS; i++) {
			ci0[i] = U_UPPERBOUND;
			ci0[N_QUADPROG_VARS + i] = -U_LOWERBOUND;
		}

		// other constraints (12, 13, 14, 15, 16, 17) are skipped
		// Custom constraint: disallow too fast changing of control
		for (int i = 0; i < N_QUADPROG_VARS - 1; i++) {
			CI[N_QUADPROG_VARS * 2 + i][i] = 1;
			CI[N_QUADPROG_VARS * 3 + i][i] = -1;

			CI[N_QUADPROG_VARS * 2 + i][i + 1] = -1;
			CI[N_QUADPROG_VARS * 3 + i][i + 1] = 1;
		}
		CI[N_QUADPROG_VARS * 3 - 1][0] = 1;
		CI[N_QUADPROG_VARS * 4 - 1][0] = -1;
		for (int i = 0; i < N_QUADPROG_VARS - 1; i++) {
			ci0[N_QUADPROG_VARS * 2 + i] = MAX_DELTA_U;
			ci0[N_QUADPROG_VARS * 3 + i] = MAX_DELTA_U;
		}
		if (!prev_u_queue.empty() && prev_u_queue.front() != STOP_SIGNAL) {
			ci0[N_QUADPROG_VARS * 3 - 1] = -prev_u_queue.front() + MAX_DELTA_U;
			ci0[N_QUADPROG_VARS * 4 - 1] = prev_u_queue.front() + MAX_DELTA_U;
		} else {
			ci0[N_QUADPROG_VARS * 3 - 1] = MAX_DELTA_U;
			ci0[N_QUADPROG_VARS * 4 - 1] = MAX_DELTA_U;
		}

		// Solve control input via quadratic programming solver
		CI = transpose(CI);
		optimal_u.resize(N_QUADPROG_VARS);
		solve_quadprog(G, g0, CE, ce0, CI, ci0, optimal_u);

		// Draw trajectory beginning from starting state
		double predicted_y = transformedImage.rows + ROBOT_OFFSET_PIXEL_Y;
		if (mpcUseDeadtimeCompensation)
			predicted_y -= compensation_steps * targetVelocity * mpcTimestep / METER_PER_PIXEL_Y;

		// Fill in and draw trajectory + robot position
		for (int y = predicted_y; y > 0; y--) {
			int x = round(polyeval(trajectory_coeffs, y));
			cv::circle(transformedFullImage, Point(x, y), 1, cv::Scalar(255, 0, 0), 1, 8, 0);
		}
		cv::circle(transformedFullImage, Point(ROBOT_POSITION_PIXEL_X, predicted_y), 8, cv::Scalar(255, 0, 0), 4, 8, 0);

		// Draw predicted MPC trajectory
		quadprogpp::Matrix<double> predicted_state = x0;
		for (int i = 0; i < mpcNumberTimesteps; i++) {
			predicted_state = A_d * predicted_state + B_d * optimal_u[i];
			predicted_y -= targetVelocity * mpcTimestep / METER_PER_PIXEL_Y;
			double predicted_x = predicted_state[0][0] / METER_PER_PIXEL_X + ROBOT_POSITION_PIXEL_X;
			cv::circle(transformedFullImage, Point(round(predicted_x), round(predicted_y)), 2, cv::Scalar(0, 0, 255), -1, 8, 0);
		}
	}

	void MPCController::processStopSign(const std_msgs::Int16ConstPtr& msg) {
		double currTime = ros::Time::now().toSec();
		if (currTime > lastStopTime + SIGN_COOLDOWN || currTime < lastStopTime + STOP_SIGN_COUNTDOWN) {
			lastStopTime = currTime;
		}
		ROS_INFO("Found stop sign: currTime: %f, lastStopTime: %f", currTime, lastStopTime);
	}

	void MPCController::processLaneSign(const std_msgs::Int16ConstPtr& msg) {
		double currTime = ros::Time::now().toSec();
		if (currTime > lastLaneTime + SIGN_COOLDOWN) {
			lastLaneTime = currTime;
			targetDrivingLane = 2 - targetDrivingLane;
		}
		ROS_INFO("Found lane sign: currTime: %f, lastTime: %f", currTime, lastLaneTime);
	}

	void MPCController::processSpeedSign(const std_msgs::Int16ConstPtr& msg) {
		double currTime = ros::Time::now().toSec();
		if (currTime > lastSpeedTime + SIGN_COOLDOWN) {
			lastSpeedTime = currTime;
			targetVelocity = 0.3f;
		}
		ROS_INFO("Found speed sign: currTime: %f, lastTime: %f", currTime, lastSpeedTime);
	}

	void MPCController::processImage(const sensor_msgs::ImageConstPtr& msg)
	{
		cv::Mat transformedImage;
		cv::Mat ThreshImageGreen, ThreshImagePink;
		std::vector<std::tuple<Point, Point, Point>> detectedLanePointTuples, usefulLanePointTuples;
		std::vector<Point> rightmostPoints, leftmostPoints, wayPoints;
		std::queue<double> prev_u_queue;
		std::vector<double> trajectory_coeffs;
		quadprogpp::Vector<double> optimal_u;

		clock_t begin = clock();
		ROS_INFO("*** NEW IMAGE ***");

		// Only use frame if time interval since last calculation has passed
		double currTime = ros::Time::now().toSec();
		if (currTime <= timeLastMPCCalc + mpcRecalcInterval)
			return;
		else
			timeLastMPCCalc = currTime;

		// Remember current control queue for deadtime compensation
		{
			std::lock_guard<std::mutex> lck(u_mutex);
			prev_u_queue = std::queue<double>(u_queue);
		}

		// Transform image
		processAndTransformImage(msg, transformedImage);
		ROS_INFO("Processed image! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// Threshold the colors to find green and pink
		inRange(transformedImage, cv::Scalar(50, 50, 120), cv::Scalar(70, 255, 255), ThreshImageGreen);
		inRange(transformedImage, cv::Scalar(155, 50, 120), cv::Scalar(175, 255, 255), ThreshImagePink);
		ROS_INFO("Thresholded image! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// Sliding window histogram peak search for lane points
		findLanePoints(transformedImage, ThreshImageGreen, ThreshImagePink, detectedLanePointTuples);
		ROS_INFO("Found lane points! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// Sort the detected lane points into leftmost, rightmost and useful points
		sortLanePoints(detectedLanePointTuples, usefulLanePointTuples, rightmostPoints, leftmostPoints);
		ROS_INFO("Sorted lane points! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// Generate waypoints from detected lane points
		generateWaypoints(transformedImage, rightmostPoints, leftmostPoints, usefulLanePointTuples, wayPoints);
		ROS_INFO("Generated waypoints! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// If we could not find more than three wayPoints, we stop and do nothing
		if (wayPoints.size() <= 3) {
			ROS_INFO("Too few waypoints, skipping! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);
			std::queue<double> empty;
			std::swap(u_queue, empty);
			u_queue.push(STOP_SIGNAL);
			return;
		}

		// Fit a polynomial x(y) as nominal trajectory
		fitTrajectoryToWaypoints(transformedImage, wayPoints, trajectory_coeffs);
		ROS_INFO("Finished fitting trajectory! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

		// Initialize full debug image
		cv::Rect helperROI(0, 0, transformedImage.cols, ROBOT_OFFSET_PIXEL_Y);
		cv::Mat transformedOriginalImage, transformedFullImage;
		cvtColor(transformedImage, transformedOriginalImage, CV_HSV2BGR);
		cv::Mat bottomFill = transformedOriginalImage(helperROI).clone();
		cv::rectangle(bottomFill, Point(0, 0), Point(transformedImage.cols, ROBOT_OFFSET_PIXEL_Y), cv::Scalar(0, 0, 0), -1);
		cv::vconcat(transformedOriginalImage, bottomFill, transformedFullImage);

		// Calculate MPC solution from trajectory and state
		solveMPCProblem(transformedImage, trajectory_coeffs, begin, prev_u_queue, transformedFullImage, optimal_u);

		// Set next control actions
		{
			std::lock_guard<std::mutex> lck(u_mutex);
			std::queue<double> empty;
			std::swap(u_queue, empty);
			for (int i = 0; i < (N_INPUTS * mpcNumberTimesteps); i++)
				u_queue.push(optimal_u[i]);

			// Make next step available
			currTimeSinceInput = mpcTimestep;
		}

		ROS_INFO("Finished MPC Optimization! t = %f, optimal_u(0) = %f", double(clock() - begin) / CLOCKS_PER_SEC, optimal_u[0]);

		if (showDebugOutputs) {
			for (int i = 0; i < (N_INPUTS * mpcNumberTimesteps); i++)
				ROS_INFO("optimal_u(%d) = %f", i, optimal_u[i]);

			// Visualization of waypoints
			for (int i = 0; i < wayPoints.size(); i++) {
				cv::circle(transformedFullImage, wayPoints.at(i), 10, cv::Scalar(0, 255, 0), 1, 8, 0);
			}

			// Transform back into original image
			cv::Mat fullImage;
			ipm->applyHomographyInv(transformedFullImage, fullImage);

			// Show debug pictures
			// cv::imshow("raw", image);
			// cv::imshow("transformed", transformedImage);
			// cv::imshow("thresholdedGreen", ThreshImageGreen);
			// cv::imshow("thresholdedPink", ThreshImagePink);
			cv::imshow("windowed", transformedFullImage);
			cv::imshow("windowedOrig", fullImage);
			ROS_INFO("Finished debug output! t = %f", double(clock() - begin) / CLOCKS_PER_SEC);

			cv::waitKey(1);
		}
	}
}
