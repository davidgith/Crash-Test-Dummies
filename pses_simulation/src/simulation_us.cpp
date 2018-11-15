/**
 * @file "simulation_us.cpp"
 * @brief Implementation simulation_us ros node.
 *
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <sensor_msgs/Range.h>
#include <yaml-cpp/yaml.h>
#include <pses_simulation/RangeSensor.h>
#include <dynamic_reconfigure/server.h>
#include <pses_simulation/RangeSensorConfig.h>

/**
 * @namespace YAML
 * @brief This namespace contains special functionality to parse from .yaml
 * files or write to .yaml files.
*/
namespace YAML
{
/**
 * @brief Functor to decode/encode a YAML::Node
*/
template <> struct convert<geometry_msgs::Pose>
{
  /**
   * @brief Encodes a pose message to YAML::Node containing x,y,z variables.
   * @param[in] rhs input pose message
   * @returns encoded YAML::Node
  */
  static Node encode(const geometry_msgs::Pose& rhs)
  {
    Node node;
    node.push_back(rhs.position.x);
    node.push_back(rhs.position.y);
    node.push_back(rhs.position.z);
    return node;
  }
  /**
   * @brief Decodes a YAML::Node containing x,y,z variables to a pose message.
   * @param[in] node input YAML::Node
   * @param[out] rhs output pose message
   * @returns true if successful, else false
   * @throws std::exception
  */
  static bool decode(const Node& node, geometry_msgs::Pose& rhs)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    rhs.position.x = node[0].as<double>();
    rhs.position.y = node[1].as<double>();
    rhs.position.z = node[2].as<double>();
    return true;
  }
};
}

/**
 * @namespace usscan
 * @brief This namespace groups all typedefs, utility functions and parameter
 * getter used in the simulation_us node
*/
namespace usscan
{
/**
 * @typedef us::scan_msg_ptr
 * @brief shortcut for a pointer to a range message
*/
typedef std::shared_ptr<sensor_msgs::Range> scan_msg_ptr;
/**
 * @typedef us::pose_msg_ptr
 * @brief shortcut for a pointer to a pose message
*/
typedef std::shared_ptr<geometry_msgs::Pose> pose_msg_ptr;
/**
 * @brief Calculate the occupancy grid postion of a given pose.
 * @param[in] laser pose message of the sensor
 * @param[in] mapInfo occupancy grid meta information
 * @return a point in image coordinates
*/
cv::Point setGridPosition(geometry_msgs::Pose& sensor,
                          nav_msgs::MapMetaData& mapInfo)
{
  unsigned int grid_x =
      (unsigned int)((sensor.position.x - mapInfo.origin.position.x) /
                     mapInfo.resolution);
  unsigned int grid_y =
      (unsigned int)((-sensor.position.y - mapInfo.origin.position.y) /
                     mapInfo.resolution);

  return cv::Point(grid_x, grid_y);
}
/**
 * @brief Get the current postion of a sensor from the tf transformation
 * service.
 * @param[in] base_frame frame id of the map coordinate system
 * @param[in] target_frame frame id of the sensor coordinate system
 * @param[in] listener tf transformation service
 * @param[out] position current position of the sensor
 * @param[out] rpy current orientation of the sensor (0: roll, 1:pitch, 2:yaw)
*/
void getPositionInfo(const std::string& base_frame,
                     const std::string& target_frame,
                     const tf::TransformListener& listener,
                     geometry_msgs::Pose* position, std::vector<double>* rpy)
{
  bool transformReady = false;
  tf::StampedTransform stf;
  tf::Stamped<tf::Pose> tmp;
  geometry_msgs::PoseStamped tmp2;
  tf::Quaternion q;

  try
  {
    transformReady = listener.waitForTransform(
        base_frame, target_frame, ros::Time(0), ros::Duration(0.01));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  if (transformReady)
  {
    try
    {
      listener.lookupTransform(base_frame, target_frame, ros::Time(0), stf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    tmp = tf::Stamped<tf::Pose>(stf, stf.stamp_, base_frame);
    tf::poseStampedTFToMsg(tmp, tmp2);
    *position = tmp2.pose;

    try
    {
      tf::quaternionMsgToTF(tmp2.pose.orientation, q);
      tf::Matrix3x3(q).getRPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
    }
    catch (std::exception ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}
/**
 * @brief Sets the RangeSensorConfig object of this sensor simulation.
 * @param[in] config dynamic reconfigure object to reconfigure this sensor
 * @param[in] level unused
 * @param[in] sensor current sensor simulation
 * @param[out] dynConfig dynamic reconfigure object to reconfigure this sensor
 *
*/
void configCallback(pses_simulation::RangeSensorConfig& config, uint32_t level,
                    rs::RangeSensor* sensor,
                    pses_simulation::RangeSensorConfig* dynConfig)
{
  sensor->setConfig(config);
  *dynConfig = config;
  ROS_DEBUG("Config was set");
}

static const double DEFAULT_LOOP_RATE = 30.0; /**< Frequency of this simulation.*/
static const std::string DEFAULT_US_RIGHT_TOPIC = "/uc_bridge/usr"; /**< Topic to which this node publishes right sensor information.*/
static const std::string DEFAULT_US_FRONT_TOPIC = "/uc_bridge/usf"; /**< Topic to which this node publishes front sensor information.*/
static const std::string DEFAULT_US_LEFT_TOPIC = "/uc_bridge/usl"; /**< Topic to which this node publishes left sensor information.*/
static const std::string DEFAULT_MAP_LOCATION =
    ros::package::getPath("pses_simulation") + "/data/map/map.pgm"; /**< Path to the occupancy grid file (map)*/
static const std::string DEFAULT_MAP_METADATA_LOCATION =
    ros::package::getPath("pses_simulation") + "/data/map/map.yaml"; /**< Path to the occupancy grid meta data
                                       file*/
static const std::string DEFAULT_US_RIGHT_FRAME_ID = "right_sensor"; /**< Coordinate system id of the right sensor*/
static const std::string DEFAULT_US_FRONT_FRAME_ID = "front_sensor"; /**< Coordinate system id of the front sensor*/
static const std::string DEFAULT_US_LEFT_FRAME_ID = "left_sensor"; /**< Coordinate system id of the left sensor*/
static const std::string DEFAULT_MAP_FRAME_ID = "map"; /**< Coordinate system id of the map*/

/**
 * @brief Get loop rate launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] loopRate loop rate in Hz
*/
void fetchLoopRate(ros::NodeHandle* nh, double& loopRate)
{
  if (nh->hasParam("us_sim/loop_rate_laserscan"))
  {
    nh->getParam("us_sim/loop_rate_laserscan", loopRate);
  }
  else
  {
    loopRate = DEFAULT_LOOP_RATE;
  }
}
/**
 * @brief Get right ultra sonic sensor topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] usrTopic topic to which this node publishes sensor
 * information.
*/
void fetchUSRTopic(ros::NodeHandle* nh, std::string& usrTopic)
{
  if (nh->hasParam("us_sim/us_right_topic"))
  {
    nh->getParam("us_sim/us_right_topic", usrTopic);
  }
  else
  {
    usrTopic = DEFAULT_US_RIGHT_TOPIC;
  }
}
/**
 * @brief Get front ultra sonic sensor topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] usfTopic topic to which this node publishes sensor
 * information.
*/
void fetchUSFTopic(ros::NodeHandle* nh, std::string& usfTopic)
{
  if (nh->hasParam("us_sim/us_front_topic"))
  {
    nh->getParam("us_sim/us_front_topic", usfTopic);
  }
  else
  {
    usfTopic = DEFAULT_US_FRONT_TOPIC;
  }
}
/**
 * @brief Get left ultra sonic sensor topic launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] uslTopic topic to which this node publishes sensor
 * information.
*/
void fetchUSLTopic(ros::NodeHandle* nh, std::string& uslTopic)
{
  if (nh->hasParam("us_sim/us_left_topic"))
  {
    nh->getParam("us_sim/us_left_topic", uslTopic);
  }
  else
  {
    uslTopic = DEFAULT_US_LEFT_TOPIC;
  }
}
/**
 * @brief Get map location launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] mapLocation path to the occupancy grid file (map)
*/
void fetchMapLocation(ros::NodeHandle* nh, std::string& mapLocation)
{
  if (nh->hasParam("us_sim/map_location"))
  {
    nh->getParam("us_sim/map_location", mapLocation);
  }
  else
  {
    mapLocation = DEFAULT_MAP_LOCATION;
  }
}
/**
 * @brief Get map meta data location launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] mapMetadataLocation path to the map meta data file
*/
void fetchMapMetadataLocation(ros::NodeHandle* nh,
                              std::string& mapMetadataLocation)
{
  if (nh->hasParam("us_sim/map_metadata_location"))
  {
    nh->getParam("us_sim/map_metadata_location", mapMetadataLocation);
  }
  else
  {
    mapMetadataLocation = DEFAULT_MAP_METADATA_LOCATION;
  }
}
/**
 * @brief Get right ultra sonic sensor frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] usrFrameID coordinate system id of this sensor
*/
void fetchUSRFrameID(ros::NodeHandle* nh, std::string& usrFrameID)
{
  if (nh->hasParam("us_sim/us_right_frame_id"))
  {
    nh->getParam("us_sim/us_right_frame_id", usrFrameID);
  }
  else
  {
    usrFrameID = DEFAULT_US_RIGHT_FRAME_ID;
  }
}
/**
 * @brief Get front ultra sonic sensor frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] usfFrameID coordinate system id of this sensor
*/
void fetchUSFFrameID(ros::NodeHandle* nh, std::string& usfFrameID)
{
  if (nh->hasParam("us_sim/us_front_frame_id"))
  {
    nh->getParam("us_sim/us_front_frame_id", usfFrameID);
  }
  else
  {
    usfFrameID = DEFAULT_US_FRONT_FRAME_ID;
  }
}
/**
 * @brief Get left ultra sonic sensor frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] uslFrameID coordinate system id of this sensor
*/
void fetchUSLFrameID(ros::NodeHandle* nh, std::string& uslFrameID)
{
  if (nh->hasParam("us_sim/us_left_frame_id"))
  {
    nh->getParam("us_sim/us_left_frame_id", uslFrameID);
  }
  else
  {
    uslFrameID = DEFAULT_US_LEFT_FRAME_ID;
  }
}
/**
 * @brief Get map frame id launch parameter.
 * @param[in] nh pointer to a ros::NodeHandle object
 * @param[out] mapFrameID coordinate system id of the map
*/
void fetchMapFrameID(ros::NodeHandle* nh, std::string& mapFrameID)
{
  if (nh->hasParam("us_sim/map_frame_id"))
  {
    nh->getParam("us_sim/map_frame_id", mapFrameID);
  }
  else
  {
    mapFrameID = DEFAULT_MAP_FRAME_ID;
  }
}
}

using namespace usscan;
/**
 * @brief Main function of this ros node.
*/
int main(int argc, char** argv)
{

  ros::init(argc, argv, "simulation_usscan");
  ros::NodeHandle nh;
  double loopRate;
  std::string usrTopic, usfTopic, uslTopic, mapLocation, mapMetadataLocation,
      usrFrameID, usfFrameID, uslFrameID, mapFrameID;

  fetchLoopRate(&nh, loopRate);
  fetchUSRTopic(&nh, usrTopic);
  fetchUSFTopic(&nh, usfTopic);
  fetchUSLTopic(&nh, uslTopic);
  fetchMapLocation(&nh, mapLocation);
  fetchMapMetadataLocation(&nh, mapMetadataLocation);
  fetchUSRFrameID(&nh, usrFrameID);
  fetchUSFFrameID(&nh, usfFrameID);
  fetchUSLFrameID(&nh, uslFrameID);
  fetchMapFrameID(&nh, mapFrameID);

  dynamic_reconfigure::Server<pses_simulation::RangeSensorConfig> server;
  dynamic_reconfigure::Server<pses_simulation::RangeSensorConfig>::CallbackType
      f;
  pses_simulation::RangeSensorConfig dynConfig;

  ros::Publisher front_us_range =
      nh.advertise<sensor_msgs::Range>(usfTopic, 10);
  ros::Publisher left_us_range = nh.advertise<sensor_msgs::Range>(uslTopic, 10);
  ros::Publisher right_us_range =
      nh.advertise<sensor_msgs::Range>(usrTopic, 10);

  sensor_msgs::Range front_range, left_range, right_range;

  // get "god" map meta info
  nav_msgs::MapMetaData mapInfo;
  YAML::Node imgMetaInfo = YAML::LoadFile(mapMetadataLocation);
  double resolution = imgMetaInfo["resolution"].as<double>();
  geometry_msgs::Pose origin = imgMetaInfo["origin"].as<geometry_msgs::Pose>();
  mapInfo.origin = origin;
  mapInfo.resolution = resolution;

  // get "god" map
  cv::Mat map = cv::imread(mapLocation, 1);
  cv::cvtColor(map, map, CV_RGB2GRAY);

  // create sensor object and link with config object
  rs::RangeSensor rs_sonar(rs::us);
  rs_sonar.setMap(map);
  rs_sonar.setMapMetaData(mapInfo);

  f = boost::bind(&configCallback, _1, _2, &rs_sonar, &dynConfig);
  server.setCallback(f);

  // variables needed for transformations
  tf::TransformListener listener;
  geometry_msgs::Pose position;
  std::vector<double> rpy(3, 0.0);
  cv::Point gridPose;

  ros::Time currentTime = ros::Time::now();

  front_range.header.frame_id = usfFrameID;
  front_range.radiation_type = 0;

  left_range.header.frame_id = uslFrameID;
  left_range.radiation_type = 0;

  right_range.header.frame_id = usrFrameID;
  right_range.radiation_type = 0;

  // Loop starts here:
  ros::Rate loop_rate(loopRate);
  while (ros::ok())
  {
    currentTime = ros::Time::now();

    front_range.header.stamp = currentTime;
    front_range.field_of_view = rs::degToRad(dynConfig.us_field_of_view);
    front_range.min_range = dynConfig.us_min_sensor_distance;
    front_range.max_range = dynConfig.us_max_sensor_distance;

    left_range.header.stamp = currentTime;
    left_range.field_of_view = rs::degToRad(dynConfig.us_field_of_view);
    left_range.min_range = dynConfig.us_min_sensor_distance;
    left_range.max_range = dynConfig.us_max_sensor_distance;

    right_range.header.stamp = currentTime;
    right_range.field_of_view = rs::degToRad(dynConfig.us_field_of_view);
    right_range.min_range = dynConfig.us_min_sensor_distance;
    right_range.max_range = dynConfig.us_max_sensor_distance;

    // front sensor
    getPositionInfo(mapFrameID, usfFrameID, listener, &position, &rpy);
    gridPose = setGridPosition(position, mapInfo);
    front_range.range = rs_sonar.getUSScan(gridPose, rs::radToDeg(rpy[2]));

    // left sensor
    getPositionInfo(mapFrameID, uslFrameID, listener, &position, &rpy);
    gridPose = setGridPosition(position, mapInfo);
    left_range.range = rs_sonar.getUSScan(gridPose, rs::radToDeg(rpy[2]));

    // right sensor
    getPositionInfo(mapFrameID, usrFrameID, listener, &position, &rpy);
    gridPose = setGridPosition(position, mapInfo);
    right_range.range = rs_sonar.getUSScan(gridPose, rs::radToDeg(rpy[2]));

    // publish sensor msgs
    front_us_range.publish(front_range);
    right_us_range.publish(right_range);
    left_us_range.publish(left_range);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
