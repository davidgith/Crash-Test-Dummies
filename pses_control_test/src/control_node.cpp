#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui.hpp>
#include <pses_control_test/ParamsConfig.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

#include "MPCController.h"

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
  f = boost::bind(&MPCController::reconfigureParameters, &controller, _1, _2);
  server.setCallback(f);

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(&MPCController::processImage, &controller, _1));
  ros::Subscriber stopSignSub = nh.subscribe<std_msgs::Int16>(
      "/sign_detection_node/StopSign", 1, boost::bind(&MPCController::processStopSign, &controller, _1));

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
    // Try getting new inputs from the controller
    if (controller.update(1.0f / update_rate_hz)) {
        steering.data = controller.getNextSteeringControl();
        motor.data = controller.getNextMotorControl();

        steeringCtrl.publish(steering);
        motorCtrl.publish(motor);
    }

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }
}
