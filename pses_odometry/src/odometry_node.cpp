#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pses_odometry/OdometryHelper.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

void imuCallback(sensor_msgs::Imu::ConstPtr msg, sensor_msgs::Imu* out){
  *out = *msg;
}

void motorCallback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* out){
  *out = *msg;
}

void hallCntCallback(std_msgs::UInt8::ConstPtr msg, std_msgs::UInt8* out){
  *out = *msg;
}

void hallDtCallback(std_msgs::Float64::ConstPtr msg, std_msgs::Float64* out){
  *out = *msg;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "odometry");

  ros::NodeHandle nh;
  // object needed to send odometric information to the navigational stack
  tf::TransformBroadcaster odomBroadcaster;
  geometry_msgs::TransformStamped odomTransform;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odomQuaternion;
  // object neeeded to store the current commands
  sensor_msgs::Imu imu;
  std_msgs::Int16 motor;
  std_msgs::UInt8 hallCnt;
  std_msgs::Float64 hallDt;
  // objects needed for odometric calculations
  OdometryHelper odomHelper;
  // Publishes the results of the odometry calculations to other ros nodes
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("/uc_bridge/imu", 10, boost::bind(imuCallback,_1,&imu));
  ros::Subscriber motorSub = nh.subscribe<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1, boost::bind(motorCallback,_1,&motor));
  ros::Subscriber hallCntSub = nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hallCntCallback,_1,&hallCnt));
  ros::Subscriber hallDtSub = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt", 10, boost::bind(hallDtCallback,_1,&hallDt));

  // Loop starts here:
  ros::Rate loop_rate(150);
  while (ros::ok())
  {
    // feed the kinematic model with sensor data, to predict location
    odomHelper.updateSensorData(imu, hallDt.data, hallCnt.data, motor.data);
    odomHelper.getQuaternion(odomQuaternion);

    // build odometry transform
    odomTransform.header.stamp = ros::Time::now();
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_footprint";
    odomTransform.transform.translation.x = odomHelper.getPosition().x;
    odomTransform.transform.translation.y = odomHelper.getPosition().y;
    odomTransform.transform.translation.z = 0.0;
    odomTransform.transform.rotation = odomQuaternion;

    // build odometry message
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position = odomHelper.getPosition();
    odom.pose.pose.orientation = odomQuaternion;
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = odomHelper.getVx();
    odom.twist.twist.linear.y = odomHelper.getVy();
    odom.twist.twist.angular.z = odomHelper.getWz();

    odomPub.publish(odom);
    odomBroadcaster.sendTransform(odomTransform);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
