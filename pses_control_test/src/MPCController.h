/**
 * @file "MPCController.h"
 * @brief MPCController class header files, containing the main controller.
 *
*/

#ifndef PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_
#define PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_

#include <bits/stdint-uintn.h>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include <mutex>
#include <queue>
#include <tuple>
#include <vector>
#include <pses_control_test/ParamsConfig.h>

#include "IPM.h"
#include "QuadProg++.hh"

using namespace cv;

namespace mpc {
	class MPCController {
	public:
		MPCController();
		virtual ~MPCController();

		bool update(float deltaTime);

		/**
		 * Returns the next steering control level.
		 * @return the next steering control level.
		 */
		int getNextSteeringControl();

		/**
		 * Returns the next motor control level.
		 * @return the next motor control level.
		 */
		int getNextMotorControl();

		/**
		 * Called on dynamic reconfiguration parameter updates. Overwrites the current configuration with the given configuration.
		 * @param config new configuration
		 * @param level not used
		 */
		void reconfigureParameters(pses_control_test::ParamsConfig &config, uint32_t level);

		/**
		 * The given image message is processed to calculate the next optimal control steps.
		 * @param msg front image message
		 */
		void processImage(const sensor_msgs::ImageConstPtr& msg);

		/**
		 * Call when a stop sign has been detected.
		 * @param msg distance to sign
		 */
		void processStopSign(const std_msgs::Int16ConstPtr& msg);

		/**
		 * Call when a lane change sign has been detected.
		 * @param msg distance to sign
		 */
		void processLaneSign(const std_msgs::Int16ConstPtr& msg);

		/**
		 * Call when a speed limit sign has been detected.
		 * @param msg distance to sign
		 */
		void processSpeedSign(const std_msgs::Int16ConstPtr& msg);

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
		int targetDrivingLane = 2;
		double targetVelocity = 0.2f;

		// Extra Configurations
		bool useDirectTrajectory = true;
		double directTrajectoryDiscount = 0;
		bool fillPinkLane = false;

		// Current transient values
		double drivingLane = 2;
		double currTimeSinceInput = 0;
		double lastStopTime = 0;
		double lastLaneTime = 0;
		double lastSpeedTime = 0;

		/**
		 * Infers waypoint x from the given triple of lane points
		 * @param leftLanePoint left lane point
		 * @param middleLanePoint middle lane point
		 * @param rightLanePoint right lane point
		 * @return waypoint inferred from the given points
		 */
		cv::Point getWaypointXFromLanePoints(const cv::Point& leftLanePoint, const cv::Point& middleLanePoint,
				const cv::Point& rightLanePoint);

		/**
		 * Infers waypoint x from outer lane points only
		 * @param leftLanePoint left lane point
		 * @param rightLanePoint right lane point
		 * @return x coordinate of inferred waypoint
		 */
		int getWaypointXFromOuterLanePoints(const cv::Point& leftLanePoint, const cv::Point& rightLanePoint);

		/**
		 * Infers waypoint x from middle lane point only
		 * @param lanePoint middle lane point
		 * @return x coordinate of inferred waypoint
		 */
		int getWaypointXFromMiddleLanePoint(const cv::Point& lanePoint);

		/**
		 * Infers waypoint x from left lane point only
		 * @param lanePoint left lane point
		 * @return x coordinate of inferred waypoint
		 */
		int getWaypointXFromLeftLanePoint(const cv::Point& lanePoint);

		/**
		 * Infers waypoint x from right lane point only
		 * @param lanePoint right lane point
		 * @return x coordinate of inferred waypoint
		 */
		 int getWaypointXFromRightLanePoint(const cv::Point& lanePoint);

		/**
		 * Preprocesses and IPM-transforms the given image
		 * @param msg raw image msg
		 * @param transformedImage IPM-transformed image
		 */
		 void processAndTransformImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& transformedImage);

		/**
		 * Finds lane points in the given image. If none exists, its x-coordinate is set to -1.
		 * @param transformedImage bird-view image
		 * @param ThreshImageGreen green-thresholded bird-view image
		 * @param ThreshImagePink pink-thresholded bird-view image
		 * @param detectedLanePoints output vector of triples of detected lanepoints
		 */
		 void findLanePoints(const cv::Mat& transformedImage, const cv::Mat& ThreshImageGreen, const cv::Mat& ThreshImagePink,
				std::vector<std::tuple<cv::Point, cv::Point, cv::Point> >& detectedLanePoints);

		/**
		 * Fills in the pink line in the given image
		 * @param transformedImage raw image
		 * @param detectedLanePoints found lane points
		 */
		 void fillPinkLine(const cv::Mat& transformedImage,
				std::vector<std::tuple<cv::Point, cv::Point, cv::Point> >& detectedLanePoints);

		/**
		 * Sorts the lane point triples into vectors
		 * @param detectedLanePoints lane point triples
		 * @param usefulPoints triples with middle point or both outer points
		 * @param rightmostPoints the rightmost points
		 * @param leftmostPoints the leftmost points
		 */
		 void sortLanePoints(std::vector<std::tuple<Point, Point, Point> > detectedLanePoints,
				std::vector<std::tuple<Point, Point, Point> >& usefulPoints, std::vector<Point>& rightmostPoints,
				std::vector<Point>& leftmostPoints);

		/**
		 * Generates waypoints from the given presorted lane points
		 * @param transformedImage debug image
		 * @param rightmostPoints the rightmost points
		 * @param leftmostPoints the leftmost points
		 * @param usefulPoints triples with middle point or both outer points
		 * @param wayPoints the vector of outputted waypoints
		 */
		 void generateWaypoints(const cv::Mat& transformedImage, std::vector<Point> rightmostPoints, std::vector<Point> leftmostPoints,
				std::vector<std::tuple<Point, Point, Point> > usefulPointTuples, std::vector<Point>& wayPoints);

		/**
		 * Fits a trajectory to the waypoints as configured.
		 * @param transformedImage debug image
		 * @param wayPoints waypoints
		 * @param trajectory_coeffs output polynomial coefficients
		 */
		 void fitTrajectoryToWaypoints(const cv::Mat& transformedImage, std::vector<Point>& wayPoints,
				std::vector<double>& trajectory_coeffs);

		/**
		 * Solves the quadratic program as configured.
		 * @param transformedImage debug image
		 * @param trajectory_coeffs polynomial coefficients
		 * @param begin time since callback start
		 * @param prev_u_queue leftover control inputs from previous iteration
		 * @param transformedFullImage debug image
		 * @param optimal_u output vector of optimal control values
		 */
		 void solveMPCProblem(const cv::Mat& transformedImage, const std::vector<double>& trajectory_coeffs, clock_t begin,
				std::queue<double> prev_u_queue, const cv::Mat& transformedFullImage, quadprogpp::Vector<double>& optimal_u);

		/**
		 * Detects leftward or rightward lane
		 * @param leftmostPoints leftmost points
		 * @param rightmostPoints rightmost points
		 * @return true if leftward lane is detected
		 */
		bool isLeftTurn(std::vector<Point>& leftmostPoints, std::vector<Point>& rightmostPoints);

		/**
		 * Finds the column maximum of the histogram closest to the center
		 * @param histogram a thresholded image
		 * @return x-coordinate of found maximum
		 */
		int findBestHistogramPeakX(cv::Mat& histogram);
	};
}

#endif /* PSES_CONTROL_TEST_SRC_MPCCONTROLLER_H_ */
