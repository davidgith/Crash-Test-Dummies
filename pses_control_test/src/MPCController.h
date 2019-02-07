/*
 * MPCController.h
 *
 *  Created on: Feb 6, 2019
 *      Author: pses
 */

#ifndef PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_
#define PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_

#include <bits/stdint-uintn.h>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>
#include <mutex>
#include <queue>
#include <tuple>
#include <vector>
#include <pses_control_test/ParamsConfig.h>

#include "IPM.h"
#include "QuadProg++.hh"

using namespace cv;

class MPCController {
public:
	MPCController();
	virtual ~MPCController();

	bool hasNewInput(float timeSinceLastInput);
	int getNextSteeringControl();
	int getNextMotorControl();
	void reconfigureParameters(pses_control_test::ParamsConfig &config, uint32_t level);
	void processInput(const sensor_msgs::ImageConstPtr& msg);

private:
	IPM* ipm;
	std::mutex u_mutex;
	std::queue<double> u_queue;
	double timeLastMPCCalc = 0;

	// Model Predictive Control Parameters
	double mpcWeightInput = 0.01f;
	double mpcWeightY = 10.0f;

	// Optimizations
	bool showDebugOutputs = true;

	// Model Predictive Control Configurations
	bool mpcUseDeadtimeCompensation = false;
	double mpcTimestep = 0.1f;
	int mpcNumberTimesteps = 20;
	double mpcRecalcInterval = 1;
	int detectionEndOffsetY = 100;

	// Drive Configurations
	bool INTERPOLATE_WITH_CURRENT_POSITION = false;
	int drivingLane = false;
	double targetVelocity = 0.2f;

	// Extra Configurations
	bool useDirectTrajectory = true;
	bool fillPinkLane = false;
	int targetMotorCtrl = 0;

	cv::Point getWaypointXFromLanePoints(const cv::Point& leftLanePoint, const cv::Point& middleLanePoint,
			const cv::Point& rightLanePoint);
	int getWaypointXFromOuterLanePoints(const cv::Point& leftLanePoint, const cv::Point& rightLanePoint);
	int getWaypointXFromMiddleLanePoint(const cv::Point& lanePoint);
	int getWaypointXFromLeftLanePoint(const cv::Point& lanePoint);
	int getWaypointXFromRightLanePoint(const cv::Point& lanePoint);
	void processAndTransformImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& transformedImage);
	void findLanePoints(const cv::Mat& transformedImage, const cv::Mat& ThreshImageGreen, const cv::Mat& ThreshImagePink,
			std::vector<std::tuple<cv::Point, cv::Point, cv::Point> >& detectedLanePoints);
	void fillPinkLine(const cv::Mat& transformedImage,
			std::vector<std::tuple<cv::Point, cv::Point, cv::Point> >& detectedLanePoints);
	void sortLanePoints(std::vector<std::tuple<Point, Point, Point> > detectedLanePoints,
			std::vector<std::tuple<Point, Point, Point> >& usefulPoints, std::vector<Point>& rightmostPoints,
			std::vector<Point>& leftmostPoints);
	void generateWaypoints(const cv::Mat& transformedImage, std::vector<Point> rightmostPoints, std::vector<Point> leftmostPoints,
			std::vector<std::tuple<Point, Point, Point> > usefulPointTuples, std::vector<Point>& wayPoints);
	void fitTrajectoryToWaypoints(const cv::Mat& transformedImage, std::vector<Point>& wayPoints,
			std::vector<double>& trajectory_coeffs);
	void solveMPCProblem(const cv::Mat& transformedImage, const std::vector<double>& trajectory_coeffs, clock_t begin,
			std::queue<double> prev_u_queue, const cv::Mat& transformedFullImage, quadprogpp::Vector<double>& optimal_u);
};

#endif /* PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_ */