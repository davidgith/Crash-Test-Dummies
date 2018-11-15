/**
 * @file "pses_simulation/CarModel.h"
 * @brief Header file for the CarModel class.
 *
*/

#ifndef CARMODEL_H_
#define CARMODEL_H_

#include <pses_simulation/ForwardKinematics.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <memory.h>
#include <ros/ros.h>

/**
 * @typedef geometry_msgs::Twist twist_msg
 * @brief shortcut for the twist message
*/
typedef geometry_msgs::Twist twist_msg;
/**
 * @typedef std::shared_ptr<std::vector<double> > pose_ptr
 * @brief shortcut for a pointer to a vector of doubles containing pose
 * information
*/
typedef std::shared_ptr<std::vector<double>> pose_ptr;
/**
 * @class CarModel CarModel.h "pses_simulation/CarModel.h"
 * @brief The CarModel class simulates kinematic behavoir of a car like mobile
 * platform, by using the ForwardKinematics library.
 *
 * This is not physically accurate simulation of forces, torque and
 *accelerations.
 * Its mainly used to demonstrate kinematic behaviour of car like robot in an
 * ideal environment, i.e. infinite acceleration, zero steering lag and perfect
 * tire grip.
 *
*/
class CarModel
{
public:
  /**
   * @brief CarModel constructor.
   * @param[in] dAxis wheel base of the mobile robot in meters.
   * @param[in] time current Ros::Time.
   * @param[in] initialPose initial pose of the robot. (0: - y-coordinate, 1:
   x-coordinate, 2: yaw in radiants).
   * @param[in] vMax maximum speed of the mobile robot in meters/second.
   * @param[in] angleMax maximum steering angle of the mobile robot in degrees.
   * @param[in] speedMax maximum motor level of the mobile robot.
   * @param[in] steeringMax maximum steering level of the mobile robot.

  */
  CarModel(const double dAxis, const ros::Time& time,
           std::vector<double> initialPose = std::vector<double>(3, 0),
           const double vMax = 2.0, const double angleMax = 22.5,
           const double speedMax = 20, const double steeringMax = 50);
  /**
   * @brief Update the simulation with current control settings and time.
   *
   * Given a current time, the simulation will calculate changes in position
   * orientation as well as speed and rotation of the robot during the time
   *interval
   * from this to the previous update.
   * @param[in] newSteering current steering level of the mobile robot.
   * @param[in] newSpeed current motor level of the mobile robot.
   * @param[in] time current Ros::Time
   * @return current position/speed/orientation/rotation of this mobile robot
  */
  const pose_ptr getUpdate(const int newSteering, const int newSpeed,
                           const ros::Time& time);
  /**
   * @brief Update the simulation with current motion parameters and time.
   *
   * Given a current time, the simulation will calculate changes in position
   * orientation as well as speed and rotation of the robot during the time
   *interval
   * from this to the previous update.
   * @param[in] cmd_vel current speed and steering angle of the robot.
   * @param[in] time current Ros::Time
   * @return current position/speed/orientation/rotation of this mobile robot
  */
  const pose_ptr getUpdateTwist(const twist_msg cmd_vel, const ros::Time& time);
  /**
   * @brief Get current steering level of the mobile plattform.
   * @return current steering level
  */
  const int getSteering() const;
  /**
   * @brief Get current steering angle of the mobile plattform.
   * @return current steering angle
  */
  const double getSteeringAngle() const;
  /**
   * @brief Get current time step.
   * @return current time step
  */
  const double getTimeStep() const;
  /**
   * @brief Get current velocity of the mobile plattform.
   * @return current velocity
  */
  const double getVelocity() const;
  /**
   * @brief Get current angular velocity of the mobile plattform.
   * @return current angular velocity
  */
  const double getAngularVelocity() const;
  /**
   * @brief Get total travelled distance of the mobile plattform.
   * @return total travelled distance
  */
  const double getDistance() const;
  /**
   * @brief Get current velocity in x direction of the mobile plattform relative
   * to the global coordinate system.
   * @return current velocity in x direction
  */
  const double getVx() const;
  /**
   * @brief Get current velocity in y direction of the mobile plattform relative
   * to the global coordinate system.
   * @return current velocity in y direction
  */
  const double getVy() const;
  /**
   * @brief Get current acceleration in x direction of the mobile plattform
   * relative to the robot coordinate system.
   * @return current acceleration in x direction
  */
  const double getAx() const;
  /**
   * @brief Get current acceleration in y direction of the mobile plattform
   * relative to the robot coordinate system.
   * @return current acceleration in y direction
  */
  const double getAy() const;

private:
  ForwardKinematics fwdKin; /**< Simulation of ackerman kinematics */
  ros::Time lastUpdate;     /**< Ros time of last update */
  double vMax;              /**< Maximum velocity in m/s */
  double angleMax;          /**< Maximum steering angle in degrees */
  double speedMax;          /**< Maximum motor level */
  double steeringMax;       /**< Maximum steering level */
  int steering;             /**< Current steering level */
  double steeringAngle;     /**< Current steering angle */
  double timeStep;          /**< Current time step */
  double velocity;          /**< Current velocity */
  double distance;          /**< Total driven distance */
  std::vector<double> pose; /**< Current pose (0: - y-coordinate, 1:
                               x-coordinate, 2: yaw in radiants) */
  double angularVelocity;   /**< Current angular velocity in rad/s */
  double v_x; /**< Current velocity in x direction relative to the global
                 coordinate system.*/
  double v_y; /**< Current velocity in y direction relative to the global
                 coordinate system.*/
  double a_x; /**< Current acceleration in x direction relative to the robot
                 coordinate system.*/
  double a_y; /**< Current acceleration in y direction relative to the robot
                 coordinate system.*/

  /**
   * @brief Conversion from current steering level to steering angle based on a
   * lookup table.
  */
  void steeringToAngle();
  /**
   * @brief Conversion from given steering angle to current steering level based
   * on a lookup table.
   * @param[in] alpha steering angle in degrees
  */
  void angleToSteering(const double alpha);
  /**
   * @brief Set current steering level
   * @param[in] steering current steering level
  */
  void setSteering(const int steering);
  /**
   * @brief Set current velocity
   * @param[in] newVel current velocity
  */
  void setVelocity(const double newVel);
  /**
   * @brief Conversion from given motor level to current velocity based on a
   * lookup table.
   * @param[in] speed motor level
  */
  void speedToVelocity(const int speed);
  /**
   * @brief Calculate and set angular velocity by providing the orientation of
   * the previous time step.
   * @param[in] oldYaw previous orientation
  */
  void setAngularVelocity(const double oldYaw);
  /**
   * @brief Calculate and set current velocity components in x and y direction
  */
  void setVelocityComponents();
  /**
   * @brief Calculate and set current acceleration components in x and y
   * direction
   * @param[in] prevV previous velocity
  */
  void setAccelerationComponents(const double prevV);
};

#endif /* CARMODEL_H_ */
