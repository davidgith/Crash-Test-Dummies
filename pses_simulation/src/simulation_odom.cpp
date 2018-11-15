/**
 * @file "simulation_odom.cpp"
 * @brief Implementation simulation_odom ros node.
 *
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <pses_simulation/CarModel.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/**
 * @namespace simulation
 * @brief This namespace groups all typedefs, utility functions and parameter
 * getter used in the simulation_odom node
*/
namespace simulation
{
/**
 * @typedef simulation::twist_msg
 * @brief shortcut for a pointer to a twist message
*/
typedef geometry_msgs::Twist twist_msg;
/**
 * @typedef simulation::int16_msg
 * @brief shortcut for a pointer to a int16 message
*/
typedef std_msgs::Int16 int16_msg;
/**
 * @typedef simulation::imu_msg
 * @brief shortcut for a pointer to a IMU message
*/
typedef sensor_msgs::Imu imu_msg;
/**
 * @typedef simulation::odom_msg
 * @brief shortcut for a pointer to a Odometry message
*/
typedef nav_msgs::Odometry odom_msg;

static const double DEFAULT_CAR_WHEELBASE = 0.25; /**< Wheelbase in meters */
static const double DEFAULT_CAR_X_COORDINATE = 0; /**< X-Coordinate in meters */
static const double DEFAULT_CAR_Y_COORDINATE = 0; /**< Y-Coordinate in meters */
static const double DEFAULT_CAR_YAW = 0; /**< Yaw in radians */
static const double DEFAULT_LOOP_RATE =
    100.0; /**< Frequency of this simulation.*/
static const std::string DEFAULT_ODOM_FRAME_ID =
    "odom"; /**< Coordinate system id of the odometry*/
static const std::string DEFAULT_ODOM_CHILD_FRAME_ID =
    "base_footprint"; /**< Base coordinate system id of the robot*/
static const std::string DEFAULT_MOTION_COMMAND_TOPIC =
    "cmd_vel"; /**< Topic from which this node gets motion commands.*/
static const std::string DEFAULT_STEERING_COMMAND_TOPIC =
    "/uc_bridge/set_steering_level_msg"; /**< Topic from which this node gets
                                            steering level commands.*/
static const std::string DEFAULT_MOTOR_COMMAND_TOPIC =
    "/uc_bridge/set_motor_level_msg"; /**< Topic from which this node gets
                                         motor level commands.*/
static const std::string DEFAULT_ODOM_TOPIC =
    "odom"; /**< Topic to which this node publishes odometry info.*/
static const std::string DEFAULT_IMU_TOPIC =
    "/uc_bridge/imu"; /**< Topic to which this node publishes IMU messages.*/

/**
 * @brief Get wheel base launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] wheelBase wheelbase in meter
*/
void fetchWheelBase(ros::NodeHandle* nh, double& wheelBase)
{
  if (nh->hasParam("odom_sim/wheel_base"))
  {
    nh->getParam("odom_sim/wheel_base", wheelBase);
  }
  else
  {
    wheelBase = DEFAULT_CAR_WHEELBASE;
  }
}

/**
 * @brief Get the initial x-coordinate of the robot.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] initialX Initial x-coordinate of the robot.
*/
void fetchInitialX(ros::NodeHandle* nh, double& initialX)
{
  if (nh->hasParam("odom_sim/initial_x_coordinate"))
  {
    nh->getParam("odom_sim/initial_x_coordinate", initialX);
  }
  else
  {
    initialX = DEFAULT_CAR_X_COORDINATE;
  }
}

/**
 * @brief Get the initial y-coordinate of the robot.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] initialY Initial y-coordinate of the robot.
*/
void fetchInitialY(ros::NodeHandle* nh, double& initialY)
{
  if (nh->hasParam("odom_sim/initial_y_coordinate"))
  {
    nh->getParam("odom_sim/initial_y_coordinate", initialY);
  }
  else
  {
    initialY = DEFAULT_CAR_Y_COORDINATE;
  }
}

/**
 * @brief Get the initial yaw of the robot. (in radians).
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] initialYaw Initial yaw of the robot.
*/
void fetchInitialYaw(ros::NodeHandle* nh, double& initialYaw)
{
  if (nh->hasParam("odom_sim/initial_yaw"))
  {
    nh->getParam("odom_sim/initial_yaw", initialYaw);
  }
  else
  {
    initialYaw = DEFAULT_CAR_YAW;
  }
}

/**
 * @brief Get loop rate launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] loopRate loop rate in Hz
*/
void fetchLoopRate(ros::NodeHandle* nh, double& loopRate)
{
  if (nh->hasParam("odom_sim/loop_rate_control"))
  {
    nh->getParam("odom_sim/loop_rate_control", loopRate);
  }
  else
  {
    loopRate = DEFAULT_LOOP_RATE;
  }
}
/**
 * @brief Get odom frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] odomFrameID coordinate system id of the odometry
*/
void fetchOdomFrameID(ros::NodeHandle* nh, std::string& odomFrameID)
{
  if (nh->hasParam("odom_sim/odom_frame_id"))
  {
    nh->getParam("odom_sim/odom_frame_id", odomFrameID);
  }
  else
  {
    odomFrameID = DEFAULT_ODOM_FRAME_ID;
  }
}
/**
 * @brief Get odom child frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] odomChildFrameID coordinate system id of the root coordinate
 * system of the robot
*/
void fetchOdomChildFrameID(ros::NodeHandle* nh, std::string& odomChildFrameID)
{
  if (nh->hasParam("odom_sim/odom_child_frame_id"))
  {
    nh->getParam("odom_sim/odom_child_frame_id", odomChildFrameID);
  }
  else
  {
    odomChildFrameID = DEFAULT_ODOM_CHILD_FRAME_ID;
  }
}
/**
 * @brief Get motion command topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] motionCommandTopic topic from which this node gets motion
 * commands
*/
void fetchMotionCommandTopic(ros::NodeHandle* nh,
                             std::string& motionCommandTopic)
{
  if (nh->hasParam("odom_sim/motion_command_topic"))
  {
    nh->getParam("odom_sim/motion_command_topic", motionCommandTopic);
  }
  else
  {
    motionCommandTopic = DEFAULT_MOTION_COMMAND_TOPIC;
  }
}
/**
 * @brief Get steering command topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] steeringCommandTopic topic from which this node gets steering
 * level commands
*/
void fetchSteeringCommandTopic(ros::NodeHandle* nh,
                               std::string& steeringCommandTopic)
{
  if (nh->hasParam("odom_sim/steering_command_topic"))
  {
    nh->getParam("odom_sim/steering_command_topic", steeringCommandTopic);
  }
  else
  {
    steeringCommandTopic = DEFAULT_STEERING_COMMAND_TOPIC;
  }
}
/**
 * @brief Get motor command topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] motorCommandTopic topic from which this node gets motor level
 * commands
*/
void fetchMotorCommandTopic(ros::NodeHandle* nh, std::string& motorCommandTopic)
{
  if (nh->hasParam("odom_sim/motor_command_topic"))
  {
    nh->getParam("odom_sim/motor_command_topic", motorCommandTopic);
  }
  else
  {
    motorCommandTopic = DEFAULT_MOTOR_COMMAND_TOPIC;
  }
}
/**
 * @brief Get odometry topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] odomTopic topic to which this node publishes odometry info
*/
void fetchOdomTopic(ros::NodeHandle* nh, std::string& odomTopic)
{
  if (nh->hasParam("odom_sim/odom_topic"))
  {
    nh->getParam("odom_sim/odom_topic", odomTopic);
  }
  else
  {
    odomTopic = DEFAULT_ODOM_TOPIC;
  }
}
/**
 * @brief Get imu topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] imuTopic topic to which this node publishes IMU messages
*/
void fetchImuTopic(ros::NodeHandle* nh, std::string& imuTopic)
{
  if (nh->hasParam("odom_sim/imu_topic"))
  {
    nh->getParam("odom_sim/imu_topic", imuTopic);
  }
  else
  {
    imuTopic = DEFAULT_IMU_TOPIC;
  }
}
/**
 * @brief Motion command topic callback
 * @param[in] cmdIn input motion message
 * @param[out] cmdOut output motion message
 * @param[out] isTwistCmd determines whether the data came from a motion command
 * or motor/steering level command.
*/
void motionCommand(const twist_msg::ConstPtr& cmdIn, twist_msg* cmdOut,
                   bool* isTwistCmd)
{
  *cmdOut = *cmdIn;
  *isTwistCmd = true;
}
/**
 * @brief Steering command topic callback
 * @param[in] cmdIn input steering level message
 * @param[out] cmdOut output steering level
 * @param[out] isTwistCmd determines whether the data came from a motion command
 * or motor/steering level command.
*/
void steeringCommand(const int16_msg::ConstPtr& cmdIn, int* cmdOut,
                     bool* isTwistCmd)
{
  *cmdOut = cmdIn->data;
  *isTwistCmd = false;
}
/**
 * @brief Motor command topic callback
 * @param[in] cmdIn input motor level message
 * @param[out] cmdOut output motor level
 * @param[out] isTwistCmd determines whether the data came from a motion command
 * or motor/steering level command.
*/
void motorCommand(const int16_msg::ConstPtr& cmdIn, int* cmdOut,
                  bool* isTwistCmd)
{
  *cmdOut = cmdIn->data;
  *isTwistCmd = false;
}
}

using namespace simulation;
/**
 * @brief Main function of this ros node.
*/
int main(int argc, char** argv)
{

  ros::init(argc, argv, "simulation_control");
  ros::NodeHandle nh;
  // create parameter variables
  double loopRate, wheelBase, initialX, initialY, initialYaw;
  std::string motionCmdTopic, steeringCmdTopic, motorCmdTopic, odomTopic,
      odomFrameID, odomChildFrameID, imuTopic;
  // fetch parameters from ros param server
  fetchLoopRate(&nh, loopRate);
  fetchWheelBase(&nh, wheelBase);
  fetchMotionCommandTopic(&nh, motionCmdTopic);
  fetchSteeringCommandTopic(&nh, steeringCmdTopic);
  fetchMotorCommandTopic(&nh, motorCmdTopic);
  fetchOdomTopic(&nh, odomTopic);
  fetchOdomFrameID(&nh, odomFrameID);
  fetchOdomChildFrameID(&nh, odomChildFrameID);
  fetchImuTopic(&nh, imuTopic);
  fetchInitialX(&nh, initialX);
  fetchInitialY(&nh, initialY);
  fetchInitialYaw(&nh, initialYaw);
  // Construct initial pose of the robot
  std::vector<double> initialPose;
  initialPose.push_back(-initialY);
  initialPose.push_back(initialX);
  initialPose.push_back(initialYaw);
  // get current time and configure our car model simulation
  ros::Time currentTime = ros::Time::now();
  CarModel car(wheelBase, currentTime, initialPose);

  std::vector<double> simPose;
  twist_msg motionCmd;
  int steeringCmd = 0, motorCmd = 0;
  bool isTwistCmd = true;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_msg odom;
  imu_msg imu;

  ros::Subscriber motionControl = nh.subscribe<twist_msg>(
      motionCmdTopic, 10,
      boost::bind(motionCommand, _1, &motionCmd, &isTwistCmd));
  ros::Subscriber steeringControl = nh.subscribe<int16_msg>(
      steeringCmdTopic, 1,
      boost::bind(steeringCommand, _1, &steeringCmd, &isTwistCmd));
  ros::Subscriber motorControl = nh.subscribe<int16_msg>(
      motorCmdTopic, 1, boost::bind(motorCommand, _1, &motorCmd, &isTwistCmd));
  ros::Publisher odomPub = nh.advertise<odom_msg>(odomTopic, 10);
  ros::Publisher imuPub = nh.advertise<imu_msg>(imuTopic, 10);

  // Loop starts here:
  ros::Rate loop_rate(loopRate);
  while (ros::ok())
  {
    currentTime = ros::Time::now();
    if (isTwistCmd == false)
    {
      simPose = *car.getUpdate(-steeringCmd / 20, motorCmd / 50, currentTime);
    }
    else
    {
      simPose = *car.getUpdateTwist(motionCmd, currentTime);
    }

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(simPose[2]);

    // first, we'll publish the transform over tf

    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = odomFrameID;
    odom_trans.child_frame_id = odomChildFrameID;

    odom_trans.transform.translation.x = simPose[1];
    odom_trans.transform.translation.y = -simPose[0];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    odom.header.stamp = currentTime;
    odom.header.frame_id = odomFrameID;
    // set the position
    odom.pose.pose.position.x = simPose[1];
    odom.pose.pose.position.y = -simPose[0];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = odomChildFrameID;
    odom.twist.twist.linear.x = car.getVx();
    odom.twist.twist.linear.y = car.getVy();
    odom.twist.twist.angular.z = car.getAngularVelocity();

    // create imu sensor message
    imu.header.stamp = currentTime;
    imu.header.frame_id = "robot_simulation";
    imu.angular_velocity.z = car.getAngularVelocity();
    imu.linear_acceleration.x = car.getAx();
    imu.linear_acceleration.y = car.getAy();

    // publish the messages
    odomPub.publish(odom);
    imuPub.publish(imu);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
