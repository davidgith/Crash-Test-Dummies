/**
 * @file "pses_simulation/RangeSensor.h"
 * @brief Header file for the rs::RangeSensor class.
 *
*/

#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <memory>
#include <pses_simulation/RangeSensorConfig.h>
#include <nav_msgs/MapMetaData.h>

/**
 * @namespace rs
 * @brief This namespace groups typedefs and enums used in the rs::RangeSensor
 * class
 * and the rs::RangeSensor class itself.
*/
namespace rs
{
/**
 * @typedef rs::rangeArray_ptr
 * @brief shortcut for a pointer to a vector of floats representing range
 * readings in meters.
*/
typedef std::shared_ptr<std::vector<float>> rangeArray_ptr;
/**
 * @enum rs::SensorType
 * @brief Enumerator for supported sensor types
*/
enum SensorType
{
  laser, /**< enum of laser sensor type */
  us     /**< enum of ultra sensonic range sensor type */
};

/**
 * @brief Calculate a correct euler angle after incrementing/decrementing.
 *
 * Problem: Euler angles are not continous, they jump from 180deg to -180 deg.
 * This function calculates new euler angles with respect to this particular
 *behaviour.
 * @param[in] theta base angle in radiants to be incremented/decremented
 * @param[in] increment amount of change to be applied to the theta value
 * @return correct yaw angle in radiants
*/
double correctYawAngle(const double theta, const double increment);
/**
 * @brief Get the sign of an interger
 * @param[in] x input integer value
 * @returns if x>=0 return +1, else -1
*/
const int sgn(const int x);
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

class RangeSensor;
}

using rs::RangeSensor;
/**
 * @class rs::RangeSensor RangeSensor.h
 * @brief The rs::RangeSensor class simulates different range finder sensors
 * based on the given rs::SensorType enumerator.
 *
 * For this simulation to work a map that represents an occupancy grid and
 * odometry information must be provided.
 *
*/
class RangeSensor
{
public:
  /**
   * @brief rs::RangeSensor constructor.
   * @param[in] type defines what kind of range finder will be simulated.
   *
  */
  RangeSensor(const SensorType& type);
  /**
   * @brief Sets the RangeSensorConfig object of this sensor simulation.
   * @param[in] config dynamic reconfigure object for this sensor
   *
  */
  void setConfig(pses_simulation::RangeSensorConfig& config);
  /**
   * @brief Sets the occupancy grid to be used by this sensor simulation.
   * @param[in] map occupancy grid represented by a monochromatic image
   *
  */
  void setMap(const cv::Mat& map);
  /**
   * @brief Sets the meta data of the occupancy grid to be used by this sensor
   *simulation.
   *
   * Meta data contains information about the occupancy grids resultion, point
   *of origin
   * and meaning of grid values.
   * @param[in] mapInfo occupancy grid meta data
   *
  */
  void setMapMetaData(const nav_msgs::MapMetaData& mapInfo);
  /**
   * @brief Get a Laserscan at the current position/orientation.
   * @param[in] sensorPos sensor position
   * @param[in] theta sensor orientation in a plane
   * @return array of range finder values in meters
   *
  */
  const rs::rangeArray_ptr getLaserScan(const cv::Point sensorPos,
                                        const double theta) const;
  /**
   * @brief Get a ultra sonic range information at the current
   *position/orientation.
   * @param[in] sensorPos sensor position
   * @param[in] theta sensor orientation in a plane
   * @return range finder value in meters
   *
  */
  const double getUSScan(const cv::Point sensorPos, const double theta) const;

private:
  /**
   * @var rs::SensorType type
   * @memberof rs::RangeSensor
   * @private
   * @brief rs::SensorType enumerator
   */
  rs::SensorType type;
  /**
   * @var pses_simulation::RangeSensorConfig config
   * @memberof rs::RangeSensor
   * @private
   * @brief dynamic reconfigure object for this sensor
   */
  pses_simulation::RangeSensorConfig config;
  /**
   * @var cv::Mat map
   * @memberof rs::RangeSensor
   * @private
   * @brief occupancy grid represented by a monochromatic image
   */
  cv::Mat map;
  /**
   * @var nav_msgs::MapMetaData mapInfo
   * @memberof rs::RangeSensor
   * @private
   * @brief occupancy grid meta data
   */
  nav_msgs::MapMetaData mapInfo;
};

#endif /* RANGESENSOR_H_ */
