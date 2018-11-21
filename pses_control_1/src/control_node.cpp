#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control_deviation)
{
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat HSVImage;
    cvtColor(image,HSVImage,CV_BGR2HSV);
    ROS_INFO("Received new image!");

    // filter green
    cv::Mat ThreshImage;
    inRange(HSVImage,cv::Scalar(50,50,120),cv::Scalar(70,255,255),ThreshImage);
    cv::imshow("view", ThreshImage);
    ROS_INFO("Shown new image!");
    //cv::waitKey(30);

    *control_deviation = 0;

    // Find control deviation in pixels (approximately linear?)
    for (int w = ThreshImage.cols-1; w >= 0; w--) {
      uchar pixel = ThreshImage.at<uchar>(ThreshImage.rows-1, w);
      if (pixel > 127) {
        *control_deviation = -250 + w;
        break;
      }
    }

    ROS_INFO("Control deviation set!");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "control_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  int control_deviation = 0;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control_deviation));

  // generate control message publisher
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("Hello world!");
  cv::namedWindow("view");
  cv::startWindowThread();

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    // P-control
    steering.data = -control_deviation;
    // publish command messages on their topics
    steeringCtrl.publish(steering);

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
