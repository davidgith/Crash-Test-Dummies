#ifndef OdometryHelper_H
#define OdometryHelper_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_odometry/ForwardKinematics.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#define WHEEL_RADIUS 0.032
#define RAD_PER_TICK 0.7853981634             // 2*PI/8
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS
#define STANDARD_GRAVITY 9.80665              // m/s^2

class OdometryHelper
{
public:
  OdometryHelper();
  void updateSensorData(const sensor_msgs::Imu& imuData, const double hallDt,
                        const unsigned char ticks, const int motor);
  void updateMotorLevel(const int motorLevel);
  const double& getYaw() const;
  const double& getRoll() const;
  const double& getPitch() const;
  const double& getSpeed() const;
  double getVx() const;
  double getVy() const;
  const double& getWz() const ;
  void getQuaternion(geometry_msgs::Quaternion& quat) const;
  const double& getDrivenDistance() const;
  const geometry_msgs::Point& getPosition() const;
  const bool& isImuCalibrated() const;

private:
  // sensor data
  double hallDt;
  sensor_msgs::Imu imuData;
  unsigned char ticks;
  unsigned int prevTicks;
  unsigned int currentTicks;
  // command data
  int motorLevel;
  int prevDirection;
  int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards

  // calculated values
  double yaw;
  double roll;
  double pitch;
  double dt;
  double speed;
  double drivenDistance;
  double deltaDistance;

  // IMU offsets
  double wxOffset;
  double wyOffset;
  double wzOffset;
  double axOffset;
  double ayOffset;
  double azOffset;

  // state variables
  uint32_t dataCount;
  bool imuCalibrated;
  ros::Time oldTimeStamp;
  geometry_msgs::Point position;
  ForwardKinematics odometric;

  void calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp);
  void calcTicks(const unsigned char ticks);
  void calibrateIMU();
  void calcRPY();
  void integrateEulerAngles(const double& dAngle, double& angle);
  void calcSpeed();
  void calcDeltaDistance();
  void calcDrivenDistance();
  void calcPosition();
};

#endif
