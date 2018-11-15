
/**
 * @file "simulation_virtual_car_info.cpp"
 * @brief Node that advertises services that provide a "virtual" car id, session id and firmware version.
 *
*/

#include <ros/ros.h>
#include <pses_ucbridge/GetFirmwareVersion.h>
#include <pses_ucbridge/GetControllerID.h>
#include <pses_ucbridge/GetSessionID.h>
/**
 * @brief A call to this Service will return a "fake" car id for simulation purposes.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested id.
 * @param[carID] desired virtual car id
 * @returns true is no errors occured, else false.
*/
bool getControllerID(pses_ucbridge::GetControllerID::Request& req,
                     pses_ucbridge::GetControllerID::Response& res, int* carID)
{
  res.answer_received = true;
  res.ID = *carID;
  
  return true;
}

/**
 * @brief A call to this Service will return a "fake" session id for simulation purposes.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested session id.
 * @returns true is no errors occured, else false.
*/
bool getSessionID(pses_ucbridge::GetSessionID::Request& req,
                  pses_ucbridge::GetSessionID::Response& res)
{
  res.answer_received = true;
  res.SID = 0;
  return true;
}

/**
 * @brief A call to this Service will return a "fake" firmware version for simulation purposes.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested firmware version.
 * @returns true is no errors occured, else false.
*/
bool getFirmwareVersion(pses_ucbridge::GetFirmwareVersion::Request& req,
                        pses_ucbridge::GetFirmwareVersion::Response& res)
{
  res.answer_received = true;
  res.version = "SIM";
  return true;
}

int main(int argc, char** argv)
{
  // set up ros node handle
  ros::init(argc, argv, "simulation_virtual_car_info", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  
  // get launch paramer
  int carID;
  nh.param<int>("/virtual_info/car_id", carID, 0);

  // register services with ros
  ros::ServiceServer getControllerIDService =
      nh.advertiseService<pses_ucbridge::GetControllerID::Request,
                          pses_ucbridge::GetControllerID::Response>(
          "/uc_bridge/get_controller_id",
          std::bind(getControllerID, std::placeholders::_1,
                    std::placeholders::_2, &carID));

  ros::ServiceServer getFirmwareVersionService =
      nh.advertiseService<pses_ucbridge::GetFirmwareVersion::Request,
                          pses_ucbridge::GetFirmwareVersion::Response>(
          "/uc_bridge/get_firmware_version",
          std::bind(getFirmwareVersion, std::placeholders::_1,
                    std::placeholders::_2));

  ros::ServiceServer getSessionIDService =
      nh.advertiseService<pses_ucbridge::GetSessionID::Request,
                          pses_ucbridge::GetSessionID::Response>(
          "/uc_bridge/get_session_id",
          std::bind(getSessionID, std::placeholders::_1,
                    std::placeholders::_2));

  ros::spin();
  return 0;
}