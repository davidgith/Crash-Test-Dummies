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

#include "MPCController.h"
#include "QuadProg++.hh"
#include "IPM.h"

#include <dynamic_reconfigure/server.h>
#include <pses_control_test/ParamsConfig.h>

int main(int argc, char** argv)
{
  // Init this node
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh;

  // MPC Controller instance
  MPCController controller;

  // Dynamic Reconfiguration
  dynamic_reconfigure::Server<pses_control_test::ParamsConfig> server;
  dynamic_reconfigure::Server<pses_control_test::ParamsConfig>::CallbackType f;
  f = boost::bind(&MPCController::callback, &controller, _1, _2);
  server.setCallback(f);

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(&MPCController::imageCallback, &controller, _1));

  // generate control message publisher
  std_msgs::Int16 motor, steering;
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  // cv::namedWindow("raw");
  // cv::namedWindow("transformed");
  // cv::namedWindow("thresholdedGreen");
  // cv::namedWindow("thresholdedPink");
  cv::namedWindow("windowed");
  cv::namedWindow("windowedOrig");
  cv::startWindowThread();

  // Loop
  int update_rate_hz = 100;
  double timeSinceApplyingLastInput = 0;
  ros::Rate loop_rate(update_rate_hz);
  while (ros::ok())
  {
    timeSinceApplyingLastInput += 1.0f / update_rate_hz;

    // Try getting new inputs from the controller
    if (controller.hasNewInput(timeSinceApplyingLastInput)) {
        steering.data = controller.getNextSteeringControl();
        motor.data = controller.getNextMotorControl();

        steeringCtrl.publish(steering);
        motorCtrl.publish(motor);

        timeSinceApplyingLastInput = 0;
    }

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }
}
