/**
 * @file "pses_ucbridge/uc_bridge.h"
 * @brief Header file for the uc_bridge node.
 *
*/

#ifndef UC_BRIDGE_H
#define UC_BRIDGE_H

#include <ros/ros.h>
#include <signal.h>
#include <pses_ucbridge/servicefunctions.h>
#include <pses_ucbridge/Communication/communication.h>
#include <pses_ucbridge/Communication/communicationconfig.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>

/**
 * @namespace uc_bridge
 * @brief This namespace contains functions used in the uc_bridge node.
 *
 * Functions in this namespace are setup/calibration functions,
 * sensor group callbacks and ros topic callbacks.
 *
*/
namespace uc_bridge
{
Communication* com_ptr; /**< Pointer to a Communication object. */
bool rstOnShutdown; /**< Should the serial device be reset before shutdown? */

/**
 * @brief This function send a soft reset command to the serial device.
 * @param[in] com Pointer to a Communication object.
*/
void resetController(Communication* com)
{
  std::string cmd = "Reset Controller";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured during initial Reset!\n Description: " << e.what());
  }
}
/**
 * @brief This function is the callback for the shutdown system signal.
 *
 * If the OS catches a shutdown system signal (e.g. ctrl+x in command line)
 * this function will be called and shut down the serial connection.
 * @param[in] sig shutdown signal
*/
void shutdownSignalHandler(int sig)
{
  if (rstOnShutdown)
  {
    resetController(com_ptr);
    ros::Duration(0.1).sleep();
  }
  try
  {
    com_ptr->stopCommunication();
    com_ptr->disconnect();
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to shut down the "
                    "connection.\n Description: "
                    << e.what());
  }
  ros::shutdown();
}
/**
 * @brief This function will trigger the registration of all predefined sensor
 * groups on the serial device.
 * @param[in] com Pointer to a Communication object.
*/
void registerSensorGroups(Communication* com)
{
  // register sensor groups
  try
  {
    if (!com->registerSensorGroups("Set Group"))
      ROS_WARN_STREAM("Registering all sensor groups failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to register sensor "
                    "groups!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will activate the motor controller of the serial device.
 * @param[in] com Pointer to a Communication object.
*/
void activateMotorController(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Drive Forward";
  short level = 0;
  input.insertParameter("speed", "int16_t", level);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating motor controller failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to activate the motor "
                    "controller!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will activate the steering controller of the serial
 * device.
 * @param[in] com Pointer to a Communication object.
*/
void activateSteeringController(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Set Steering Level";
  short level = 0;
  input.insertParameter("steering", "int16_t", level);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating steering controller failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to activate the steering "
                    "controller!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will activate the kinect power supply of the serial
 * device.
 * @param[in] com Pointer to a Communication object.
*/
void activateKinect(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Toggle Kinect On";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating kinect failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate kinect!\n Description: "
        << e.what());
  }
}
/**
 * @brief This function will activate the ultra sonic range sensors of the
 * serial device.
 * @param[in] com Pointer to a Communication object.
*/
void activateUS(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Toggle US On";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating us-sensors failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate us-sensors!\n Description: "
        << e.what());
  }
}
/**
 * @brief This function will activate the daq of the serial device.
 * @param[in] com Pointer to a Communication object.
*/
void activateDAQ(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Start DAQ";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating daq failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate daq!\n Description: "
        << e.what());
  }
}

// sensor group callbacks
/**
 * @brief This function will be called whenever a message from sensor group 1
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 1 contains all ultra sonic range sensor readings.
 * @param[in] grp Pointer to SensorGroup 1 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage1(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::Range usl, usr, usf;
  double l, r, f;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("USL", l);
    grp->getChannelValueConverted("USF", f);
    grp->getChannelValueConverted("USR", r);
    usl.range = l;
    usf.range = f;
    usr.range = r;

    usl.max_range = 3;
    usl.min_range = 0.06;
    usl.field_of_view = 0.76;
    usl.radiation_type = 0;
    usl.header.frame_id = "left_sensor";
    usl.header.stamp = t;

    usf.max_range = 3;
    usf.min_range = 0.06;
    usf.field_of_view = 0.76;
    usf.radiation_type = 0;
    usf.header.frame_id = "front_sensor";
    usf.header.stamp = t;

    usr.max_range = 3;
    usr.min_range = 0.06;
    usr.field_of_view = 0.76;
    usr.radiation_type = 0;
    usr.header.frame_id = "right_sensor";
    usr.header.stamp = t;

    pub["USL"]->publish(usl);
    pub["USF"]->publish(usf);
    pub["USR"]->publish(usr);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp1' occured!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will be called whenever a message from sensor group 2
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 2 contains all IMU sensor readings.
 * @param[in] grp Pointer to SensorGroup 2 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage2(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::Imu imu;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("GX", imu.angular_velocity.x);
    grp->getChannelValueConverted("GY", imu.angular_velocity.y);
    grp->getChannelValueConverted("GZ", imu.angular_velocity.z);
    grp->getChannelValueConverted("AX", imu.linear_acceleration.x);
    grp->getChannelValueConverted("AY", imu.linear_acceleration.y);
    grp->getChannelValueConverted("AZ", imu.linear_acceleration.z);
    imu.header.stamp = t;
    imu.header.frame_id = "robot_imu";

    pub["IMU"]->publish(imu);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp2' occured!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will be called whenever a message from sensor group 3
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 3 contains all hall sensor readings (makshift rotary encoder).
 * @param[in] grp Pointer to SensorGroup 3 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage3(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  std_msgs::UInt8 hallcnt;
  std_msgs::Float64 halldt, halldt8;
  try
  {
    grp->getChannelValue("HALL_CNT", hallcnt.data);
    grp->getChannelValueConverted("HALL_DT", halldt.data);
    pub["HALL_CNT"]->publish(hallcnt);
    pub["HALL_DT"]->publish(halldt);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp3' occured!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will be called whenever a message from sensor group 4
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 4 contains all magnetic field sensor readings.
 * @param[in] grp Pointer to SensorGroup 4 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage4(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::MagneticField mag;
  // short mx, my, mz;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("MX", mag.magnetic_field.x);
    grp->getChannelValueConverted("MY", mag.magnetic_field.y);
    grp->getChannelValueConverted("MZ", mag.magnetic_field.z);
    mag.header.stamp = t;
    mag.header.frame_id = "robot_magnetometer";
    pub["MAG"]->publish(mag);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp4' occured!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will be called whenever a message from sensor group 5
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 5 contains all battery readings.
 * @param[in] grp Pointer to SensorGroup 5 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage5(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::BatteryState batVD, batVS;
  double vsbat, vdbat;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("VDBAT", vdbat);
    grp->getChannelValueConverted("VSBAT", vsbat);
    batVD.voltage = vdbat;
    batVS.voltage = vsbat;
    batVD.header.stamp = t;
    batVS.header.stamp = t;

    pub["VDBAT"]->publish(batVD);
    pub["VSBAT"]->publish(batVS);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp5' occured!\n Description: "
                    << e.what());
  }
}
/**
 * @brief This function will be called whenever a message from sensor group 6
 * has been received and publish the sensor data on ros topcis.
 *
 * Sensor group 6 contains hall sensor readings for time between 8 impulses.
 * @param[in] grp Pointer to SensorGroup 6 object
 * @param[in] map of ros::Publisher objects.
*/
void publishSensorGroupMessage6(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  std_msgs::UInt8 hallcnt;
  std_msgs::Float64 halldt, halldt8;
  try
  {
    grp->getChannelValueConverted("HALL_DT8", halldt8.data);
    pub["HALL_DT8"]->publish(halldt8);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp6' occured!\n Description: "
                    << e.what());
  }
}

// debug/error/text callbacks
/**
 * @brief This function will be called whenever a communication error occured
 * and post its content as a ROS Warning.
 * @param[in] msg error message
*/
void errorCallback(const std::string& msg)
{
  ROS_WARN_STREAM("A communication Error occured!\n" << msg);
}
/**
 * @brief This function will be called whenever the communication received a
 * plain text message which will be posted as ROS Info.
 * @param[in] msg plain text message
*/
void textCallback(const std::string& msg)
{
  ROS_INFO_STREAM("UC board sent the following info:\n" << msg);
}
/**
 * @brief This function will be called whenever the communication received any
 * message if the debug mode is enabled.
 *
 * The content of the debug message will be published on a ros topic.
 * @param[in] msg plain debug message
 * @param[out] pub ros::Publisher for the debug topic
*/
void publishDebugMessage(const std::string& msg, ros::Publisher* pub)
{
  std_msgs::String debug;
  debug.data = msg;
  pub->publish(debug);
}

// ros command message callbacks
/**
 * @brief This function will be called whenever a ros node published a message
 * on the motor level topic.
 *
 * The content of the motor level message determines what motor level will be set.
 * This is an alternative way to ros services for controlling the robot.
 * @param[in] motorLevel motor level message containing the motor level to be set.
 * @param[in] com Pointer to a Communication object.
*/
void motorLevelCallback(std_msgs::Int16::ConstPtr motorLevel,
                        Communication* com)
{
  bool was_set = false;
  std::string cmd = "Drive Forward";
  short level = motorLevel->data;
  if (level < 0)
  {
    cmd = "Drive Backward";
    level = -level;
  }
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("speed", "int16_t", level);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Message 'set_motor_level_msg' occured!\n Description: "
        << e.what());
    was_set = false;
  }
  if (!was_set)
    ROS_WARN_STREAM("Motor level set to: " << motorLevel->data << " failed!");
}
/**
 * @brief This function will be called whenever a ros node published a message
 * on the steering level topic.
 *
 * The content of the steering level message determines what motor level will be set.
 * This is an alternative way to ros services for controlling the robot.
 * @param[in] steeringLevel steering level message containing the steering level to be set.
 * @param[in] com Pointer to a Communication object.
*/
void steeringLevelCallback(std_msgs::Int16::ConstPtr steeringLevel,
                           Communication* com)
{
  bool was_set = false;
  std::string cmd = "Set Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("steering", "int16_t", steeringLevel->data);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Message 'set_steering_level_msg' occured!\n Description: "
        << e.what());
    was_set = false;
  }
  if (!was_set)
    ROS_WARN_STREAM("Steering level set to: " << steeringLevel->data
                                              << " failed!");
}

// debug raw message callback
/**
 * @brief This function will be called whenever a ros node published a message
 * on the raw communication topic.
 *
 * The content of the message will be transmitted directly to the serial device.
 * This is meant as a direct way to communicate with the serial device for debugging purposes.
 * @param[in] msg message to be transmitted
 * @param[in] com Pointer to a Communication object.
*/
void ucBoardMessageCallback(std_msgs::String::ConstPtr msg, Communication* com)
{
  try
  {
    com->sendRawMessage(msg->data);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Message 'send_uc_board_msg' occured!\n Description: "
        << e.what());
  }
}
}

#endif // UC_BRIDGE_H
