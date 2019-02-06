/*
 * MPCController.h
 *
 *  Created on: Feb 6, 2019
 *      Author: pses
 */

#ifndef PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_
#define PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_

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
#include <queue>
#include <mutex>

#include "QuadProg++.hh"
#include "IPM.h"

#include <dynamic_reconfigure/server.h>
#include <pses_control_test/ParamsConfig.h>

using namespace cv;
using namespace std;
using namespace Eigen;

class MPCController {
public:
	MPCController();
	virtual ~MPCController();
	bool hasNewInput(float timeSinceLastInput);
	int getNextSteeringControl();
	int getNextMotorControl();
	void callback(pses_control_test::ParamsConfig &config, uint32_t level);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
	IPM* ipm;
	std::mutex u_mutex;
	std::queue<double> u_queue;
	double timeLastMPCCalc = 0;

	// Model Predictive Control Parameters
	double MPC_WEIGHT_U = 0.01f;
	double MPC_WEIGHT_Y = 10.0f;

	// Optimizations
	bool DEBUG_OUTPUT = true;

	// Model Predictive Control Configurations
	bool MPC_DEADTIME_COMPENSATION = false;
	double MPC_DT = 0.1f;
	int MPC_STEPS = 20;
	double MPC_RECALC_INTERVAL = 1;
	int DETECTION_END_OFFSET_Y = 100;

	// Drive Configurations
	bool INTERPOLATE_WITH_CURRENT_POSITION = false;
	int DRIVING_LANE = false;
	double TARGET_VELOCITY = 0.2f;

	// Extra Configurations
	bool DIRECT_TRAJECTORY = true;
	bool FILL_PINK_LANE = false;
	int MOTOR_CTRL = 0;


	Point getWaypointXFromLanePoints(const Point& leftLanePoint,
			const Point& middleLanePoint, const Point& rightLanePoint);
	int getWaypointXFromOuterLanePoints(const Point& leftLanePoint,
			const Point& rightLanePoint);
	int getWaypointXFromMiddleLanePoint(const Point& lanePoint);
	int getWaypointXFromLeftLanePoint(const Point& lanePoint);
	int getWaypointXFromRightLanePoint(const Point& lanePoint) ;
};

#endif /* PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_ */
