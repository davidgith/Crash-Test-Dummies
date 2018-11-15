/**
 * @file "pses_simulation/ForwardKinematics.h"
 * @brief Header file for the ForwardKinematics class.
 *
*/

#ifndef FORWARDKINEMATICS_H_
#define FORWARDKINEMATICS_H_

#include <utility>
#include <vector>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <cmath>

/**
 * @class ForwardKinematics ForwardKinematics.h
 * @brief The ForwardKinematics class simulates kinematic behavoir of a car like mobile
 * platform given its current control parameters (e.g. steering, speed etc.)
 *
 * This is not physically accurate simulation of forces, torque and
 *accelerations.
 * Its mainly used to demonstrate kinematic behaviour of car like robot in an
 * ideal environment, i.e. infinite acceleration, zero steering lag and perfect
 * tire grip.
 *
*/
class ForwardKinematics
{
public:
  /**
   * @brief ForwardKinematics default constructor.
  */
  ForwardKinematics();
  /**
   * @brief ForwardKinematics default copy constructor.
  */
  ForwardKinematics(const ForwardKinematics& other);
  /**
   * @brief ForwardKinematics constructor.
   * @param[in] k wheel base of the mobile robot in meters.
   * @param[in] initialPose initial pose of the robot. (0: - y-coordinate, 1: x-coordinate, 2: yaw in radiants)
  */
  ForwardKinematics(double k, std::vector<double> initialPose = std::vector<double>(3, 0));
  /**
   * @brief Get the radius of the current robot tracjectory around the instantaneous center of curvature (ICC).
   * @returns radius in meters
  */
  const double getRadius() const;
  /**
   * @brief Rounds values close to zero to exactly zero.
   * @returns 0 if value > epsilon, else value.
  */
  double flattenZeros(double value);
  /**
   * @brief Conversion from degrees to radiants.
   * @param[in] angle angle in degrees
   * @returns angle in radiants
  */
  double degToRad(double angle);
  /**
   * @brief Conversion from radiants to degrees.
   * @param[in] angle angle in radiants
   * @returns angle in degrees
  */
  double radToDeg(double angle);
  /**
   * @brief Calculates the current instantaneous center of curvature given a steering angle.
   * @param[in] alpha current steering angle
   * @returns pair of coordinates of the ICC (first: x, second: y)
  */
  std::pair<double, double> calcICC(double alpha);
  /**
   * @brief Calculate the radius of the current robot tracjectory around the instantaneous center of curvature (ICC).
   * @param[in] ICC pair of coordinates of the ICC (first: x, second: y)
   * @returns radius in meters
  */
  double calcRadius(std::pair<double, double> ICC);
  /**
   * @brief Calculate the current orientation of the robot with respect to the coordinate system of the previous update.
   * @param[in] distance distance travelled between updates
   * @param[in] radius radius with respect to the ICC in meters
   * @returns current orientation in radiants
  */
  double calcTheta(double distance, double radius);
  /**
   * @brief Calculate the current rotation matrix.
   * @param[in] theta orientation of the previous update
   * @param[in] alpha current steering angle
   * @returns current rotation matrix
  */
  Eigen::Matrix4d* calcRot(double theta, double alpha);
  /**
   * @brief Calculate the current translation matrix.
   * @param[in] alpha current steering angle
   * @param[in] distance distance travelled between updates
   * @param[in] radius radius with respect to the ICC in meters
   * @param[in] ICC pair of coordinates of the ICC (first: x, second: y)
   * @returns current translation matrix
  */
  Eigen::Matrix4d* calcTrans(double radius, double distance, double alpha,
                             std::pair<double, double> ICC);
  /**
   * @brief Get the current position and orientation.
   * @param[in] alpha current steering angle
   * @param[in] distance distance travelled between updates
   * @returns current Current pose (0: - y-coordinate, 1: x-coordinate, 2: yaw in radiants)
  */
  std::vector<double>& getUpdate(double alpha, double distance);
  double PI; /**< pi */

private:
  double k; /**< Wheelbase in meters */
  double radius; /**< Radius of the current robot tracjectory around the instantaneous center of curvature (ICC) */
  Eigen::Matrix4d initT; /**< Initial forward kinematic transformation matrix */
  std::vector<Eigen::Matrix4d> T; /**< History of kinematic transformation matrices */
  Eigen::Matrix4d prevT; /**< Previous kinematic transformation matrix */
  std::vector<double> currentPosition; /**< current Current pose (0: - y-coordinate, 1: x-coordinate, 2: yaw in radiants) */
  /**
   * @brief Initialize simulation.
  */
  void init();
};

#endif /* FORWARDKINEMATICS_H_ */
