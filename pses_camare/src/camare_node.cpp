#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include "opencv2/opencv.hpp"

// gets called whenever a new message is availible in the input puffer

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "helloworld_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  sensor_msgs::Image image;

  //using webcam to get a picture
  

  // generate control message publisher
  ros::Publisher camareImage =
      nh.advertise<sensor_msgs::Image>("camare_image", 10);

  ROS_INFO("I got picture!");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    // publish command messages on their topics
    camareImage.publish(image);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
