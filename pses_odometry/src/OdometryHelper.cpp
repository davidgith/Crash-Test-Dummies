#include <pses_odometry/OdometryHelper.h>

OdometryHelper::OdometryHelper()
{
  ticks = 0;
  prevTicks = 0;
  currentTicks = 0;
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
  dt = 0.0;
  drivenDistance = 0.0;
  speed = 0.0;
  drivingDirection = 0;
  prevDirection = 0;
  wxOffset = 0.0;
  wyOffset = 0.0;
  wzOffset = 0.0;
  axOffset = 0.0;
  ayOffset = 0.0;
  azOffset = 0.0;
  dataCount = 0;
  imuCalibrated = false;
  oldTimeStamp = ros::Time::now();
  odometric.setK(
      0.25); // set distance between front axis and back axis (in meters)
}

void OdometryHelper::updateSensorData(const sensor_msgs::Imu& imuData,
                                      const double hallDt, const unsigned char ticks, 
                                      const int motor)
{
  calcDt(imuData.header.stamp, oldTimeStamp);
  calcTicks(ticks);
  if (imuCalibrated)
  {
    this->imuData.angular_velocity.x = imuData.angular_velocity.x - wxOffset;
    this->imuData.angular_velocity.y = imuData.angular_velocity.y - wyOffset;
    this->imuData.angular_velocity.z = imuData.angular_velocity.z - wzOffset;
    this->imuData.linear_acceleration.x =
        imuData.linear_acceleration.x - axOffset;
    this->imuData.linear_acceleration.y =
        imuData.linear_acceleration.y - ayOffset;
    this->imuData.linear_acceleration.z =
        imuData.linear_acceleration.z - azOffset;
    calcRPY();
  }
  else
  {
    this->imuData = imuData;
    calibrateIMU();
    //return;
  }
  this->hallDt = hallDt;
  updateMotorLevel(motor);
  calcSpeed();
  calcDeltaDistance();
  calcDrivenDistance();
  calcPosition();
  oldTimeStamp = imuData.header.stamp;
}

void OdometryHelper::updateMotorLevel(const int motorLevel)
{
  this->motorLevel = motorLevel;
  prevDirection = drivingDirection;
  if (motorLevel < 0)
    drivingDirection = -1;
  else if (motorLevel > 0)
    drivingDirection = 1;
  else
    drivingDirection = 0;
}

const double& OdometryHelper::getYaw() const { return yaw; }

const double& OdometryHelper::getRoll() const { return roll; }

const double& OdometryHelper::getPitch() const { return pitch; }

const double& OdometryHelper::getSpeed() const { return speed; }

double OdometryHelper::getVx() const { return speed * std::cos(yaw); }

double OdometryHelper::getVy() const { return speed * std::sin(yaw); }

const double& OdometryHelper::getWz() const { return imuData.angular_velocity.z; }

void OdometryHelper::getQuaternion(geometry_msgs::Quaternion& quat) const
{
  quat = tf::createQuaternionMsgFromYaw(yaw);
}

const double& OdometryHelper::getDrivenDistance() const { return drivenDistance; }

const geometry_msgs::Point& OdometryHelper::getPosition() const { return position; }

const bool& OdometryHelper::isImuCalibrated() const { return imuCalibrated; }

void OdometryHelper::calcDt(const ros::Time& currentTimeStamp,
                            const ros::Time& oldTimeStamp)
{
  dt = (currentTimeStamp - oldTimeStamp).toSec();
}

void OdometryHelper::calcTicks(const unsigned char ticks) {
  if (ticks < this->ticks) 
  {
    currentTicks = currentTicks + (256 - this->ticks) + ticks;
  }
  else
  {
    currentTicks = currentTicks + (ticks - this->ticks);
  }
  this->ticks = ticks;
}

void OdometryHelper::calibrateIMU()
{
  if (dataCount < 150 && imuData.angular_velocity.z != 0)
  {
    dataCount++;
    wxOffset += imuData.angular_velocity.x;
    wyOffset += imuData.angular_velocity.y;
    wzOffset += imuData.angular_velocity.z;
    axOffset += imuData.linear_acceleration.x;
    ayOffset += imuData.linear_acceleration.x;
    azOffset += (imuData.linear_acceleration.z - STANDARD_GRAVITY);
  }
  else if (dataCount == 150 && !imuCalibrated)
  {
    wxOffset /= dataCount;
    wyOffset /= dataCount;
    wzOffset /= dataCount;
    axOffset /= dataCount;
    ayOffset /= dataCount;
    azOffset /= dataCount;
    imuCalibrated = true;
  }
}

void OdometryHelper::calcRPY() {
  integrateEulerAngles(imuData.angular_velocity.x, roll);
  integrateEulerAngles(imuData.angular_velocity.y, pitch);
  integrateEulerAngles(imuData.angular_velocity.z, yaw);
}

void OdometryHelper::integrateEulerAngles(const double& dAngle, double& angle)
{
  double result = angle + dAngle * dt;
  if (result > M_PI)
  {
    angle = -2 * M_PI + result;
  }
  else if (result < -M_PI)
  {
    angle = 2 * M_PI + result;
  }
  else
  {
    angle = result;
  }
}

void OdometryHelper::calcSpeed()
{
  if (!std::isnan(hallDt))
  {
    speed = drivingDirection * DRIVEN_DISTANCE_PER_TICK / hallDt;
  }
  else
  {
    if (prevDirection != drivingDirection)
      speed = 0;
    else if (drivingDirection == 0)
      speed = 0;
  }
}

void OdometryHelper::calcDeltaDistance()
{
  //deltaDistance =
  //    std::isnan(hallDt) ? 0.0 : drivingDirection * DRIVEN_DISTANCE_PER_TICK;
  if (std::isnan(hallDt))
  {
    deltaDistance = 0.0;
    return;
  }
  deltaDistance = drivingDirection * static_cast<long>(currentTicks - prevTicks) * DRIVEN_DISTANCE_PER_TICK;
  prevTicks = currentTicks;
}

void OdometryHelper::calcDrivenDistance()
{
  drivenDistance = drivenDistance + deltaDistance;
}
void OdometryHelper::calcPosition()
{
  std::vector<double> pos = odometric.getUpdateWithGyro(yaw, deltaDistance);
  position.x = pos.at(1);
  position.y = -pos.at(0);
  position.z = 0.0;
}
