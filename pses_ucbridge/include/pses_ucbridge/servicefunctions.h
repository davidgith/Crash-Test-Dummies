/**
 * @file "pses_ucbridge/servicefunctions.h"
 * @brief Contains all service functions and their respective implementations.
 *
*/

#ifndef SERVICEFUNCTIONS_H
#define SERVICEFUNCTIONS_H

#include <ros/ros.h>
#include <pses_ucbridge/Communication/communication.h>
// service includes begin
#include <pses_ucbridge/DeleteGroup.h>
#include <pses_ucbridge/GetControllerID.h>
#include <pses_ucbridge/GetDAQStatus.h>
#include <pses_ucbridge/GetFirmwareVersion.h>
#include <pses_ucbridge/GetIMUInfo.h>
#include <pses_ucbridge/GetInfoAllGroups.h>
#include <pses_ucbridge/GetInfoGroup.h>
#include <pses_ucbridge/GetKinectStatus.h>
#include <pses_ucbridge/GetMagASA.h>
#include <pses_ucbridge/GetMagInfo.h>
#include <pses_ucbridge/GetMotorLevel.h>
#include <pses_ucbridge/GetSessionID.h>
#include <pses_ucbridge/GetSteeringLevel.h>
#include <pses_ucbridge/GetUSInfo.h>
#include <pses_ucbridge/ResetController.h>
#include <pses_ucbridge/ResetDMS.h>
#include <pses_ucbridge/SetIMU.h>
#include <pses_ucbridge/SetMag.h>
#include <pses_ucbridge/SetMotorLevel.h>
#include <pses_ucbridge/SetSteering.h>
#include <pses_ucbridge/SetSessionID.h>
#include <pses_ucbridge/SetUS.h>
#include <pses_ucbridge/ToggleBrakes.h>
#include <pses_ucbridge/ToggleDAQ.h>
#include <pses_ucbridge/ToggleDMS.h>
#include <pses_ucbridge/ToggleGroup.h>
#include <pses_ucbridge/ToggleKinect.h>
#include <pses_ucbridge/ToggleMotor.h>
#include <pses_ucbridge/ToggleUS.h>

// service includes end

/**
 * @namespace ServiceFunctions
 * @brief Contains all service functions and their respective implementations.
 *
 * Service functions get called whenever a ROS node calls a Service.
 *
*/
namespace ServiceFunctions
{

/**
 * @brief A call to this Service will send a command to the serial device
 * to delete a certain group.
 * @param[in] req Request message defines which group should be deleted.
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool deleteGroup(pses_ucbridge::DeleteGroup::Request& req,
                 pses_ucbridge::DeleteGroup::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Delete Group";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Delete Group' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit its id.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested id.
 * @returns true is no errors occured, else false.
*/
bool getControllerID(pses_ucbridge::GetControllerID::Request& req,
                     pses_ucbridge::GetControllerID::Response& res,
                     Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Controller ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("ID", res.ID);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Controller ID' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the daq status.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested daq status.
 * @returns true is no errors occured, else false.
*/
bool getDAQStatus(pses_ucbridge::GetDAQStatus::Request& req,
                  pses_ucbridge::GetDAQStatus::Response& res,
                  Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Is DAQ Started";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.status);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Is DAQ Started' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit its firmware version.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested firmware version.
 * @returns true is no errors occured, else false.
*/
bool getFirmwareVersion(pses_ucbridge::GetFirmwareVersion::Request& req,
                        pses_ucbridge::GetFirmwareVersion::Response& res,
                        Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Firmware Version";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("version", res.version);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Firmware Version' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the calibration of its IMU.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested IMU info.
 * @returns true is no errors occured, else false.
*/
bool getIMUInfo(pses_ucbridge::GetIMUInfo::Request& req,
                pses_ucbridge::GetIMUInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get IMU Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.info);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get IMU info' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit info about all registered groups.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested group info.
 * @returns true is no errors occured, else false.
*/
bool getInfoAllGroups(pses_ucbridge::GetInfoAllGroups::Request& req,
                      pses_ucbridge::GetInfoAllGroups::Response& res,
                      Communication* com)
{
  res.answer_received = false;
  std::string cmd = "All Groups Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.info);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'All Groups Info' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the info about a particular group.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested group info.
 * @returns true is no errors occured, else false.
*/
bool getInfoGroup(pses_ucbridge::GetInfoGroup::Request& req,
                  pses_ucbridge::GetInfoGroup::Response& res,
                  Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Group Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.info);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Service 'Group Info' occured!\n Description: "
                    << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the power supply status of the kinect.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested kinect status.
 * @returns true is no errors occured, else false.
*/
bool getKinectStatus(pses_ucbridge::GetKinectStatus::Request& req,
                     pses_ucbridge::GetKinectStatus::Response& res,
                     Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Kinect Status";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("kinect_stat", res.status);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Kinect Status' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the calibration of its magnetic field sensor.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested Mag info.
 * @returns true is no errors occured, else false.
*/
bool getMagASA(pses_ucbridge::GetMagASA::Request& req,
               pses_ucbridge::GetMagASA::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get MAG ASA";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("asa_mx", res.asa_mx);
    output.getParameterValue("asa_my", res.asa_my);
    output.getParameterValue("asa_mz", res.asa_mz);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Service 'Get Mag ASA' occured!\n Description: "
                    << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit info about its magnetic field sensor.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested Mag info.
 * @returns true is no errors occured, else false.
*/
bool getMagInfo(pses_ucbridge::GetMagInfo::Request& req,
                pses_ucbridge::GetMagInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get MAG Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.info);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Mag info' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the current motor level.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested motor level info.
 * @returns true is no errors occured, else false.
*/
bool getMotorLevel(pses_ucbridge::GetMotorLevel::Request& req,
                   pses_ucbridge::GetMotorLevel::Response& res,
                   Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Motor Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    std::string direction;
    short level;
    output.getParameterValue("direction", direction);
    output.getParameterValue("level", level);
    if (direction.compare("F") != 0)
    {
      level = -level;
    }
    res.level = level;
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Motor Level' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit its current session id.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested session id.
 * @returns true is no errors occured, else false.
*/
bool getSessionID(pses_ucbridge::GetSessionID::Request& req,
                  pses_ucbridge::GetSessionID::Response& res,
                  Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("ID", res.SID);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Session ID' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the current steering level.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested steering level info.
 * @returns true is no errors occured, else false.
*/
bool getSteeringLevel(pses_ucbridge::GetSteeringLevel::Request& req,
                      pses_ucbridge::GetSteeringLevel::Response& res,
                      Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("level", res.level);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Get Steering Level' occured!\n Description: "
        << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to transmit the calibration of its ultra sonic range sensors.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call and the requested ultra sonic range sensor info.
 * @returns true is no errors occured, else false.
*/
bool getUSInfo(pses_ucbridge::GetUSInfo::Request& req,
               pses_ucbridge::GetUSInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get US Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.answer_received = com->sendCommand(cmd, input, output);
    if (!res.answer_received)
      return false;
    output.getParameterValue("info", res.info);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Service 'Get US info' occured!\n Description: "
                    << e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to do perform a soft reset.
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool resetController(pses_ucbridge::ResetController::Request& req,
                     pses_ucbridge::ResetController::Response& res,
                     Communication* com)
{
  res.was_set = false;
  std::string cmd = "Reset Controller";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    com->sendCommand(cmd, input, output);
    res.was_set = true;
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Reset Controller' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to reset the counter on the dead man's switch
 * @param[in] req Request message is empty.
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool resetDMS(pses_ucbridge::ResetDMS::Request& req,
              pses_ucbridge::ResetDMS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Reset DMS";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'toggleDMS' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to change the calibration of its IMU.
 * @param[in] req Request message contains the new accel range, accel filter, gyro range and gyro filter values..
 * @param[out] res Response message informs the calling process about the
 *success of the service call and adds the new calibration from the imu.
 * @returns true is no errors occured, else false.
*/
bool setIMU(pses_ucbridge::SetIMU::Request& req,
            pses_ucbridge::SetIMU::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set IMU";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("arange", "int16_t", req.accel_range);
  input.insertParameter("afilt", "int16_t", req.accel_filter);
  input.insertParameter("grange", "int16_t", req.gyro_range);
  input.insertParameter("gfilt", "int16_t", req.gyro_filter);
  std::vector<std::string> options;
  options.push_back("IMU_AccelRange");
  options.push_back("IMU_AccelFilter");
  options.push_back("IMU_GyroRange");
  options.push_back("IMU_GyroFilter");
  options.push_back("IMU_si_info");
  try
  {
    res.was_set = com->sendCommand(cmd, options, input, output);
    output.getParameterValue("info", res.in_si_units);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set IMU' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * whether to use the magnetic field sensor calibration or not.
 * @param[in] req Request message contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool setMag(pses_ucbridge::SetMag::Request& req,
            pses_ucbridge::SetMag::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set MAG";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  unsigned char use_asa = req.use_asa;
  input.insertParameter("use_asa", "uint8_t", use_asa);
  std::vector<std::string> options;
  options.push_back("MAG_USEASA");
  try
  {
    res.was_set = com->sendCommand(cmd, options, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set IMU' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to change the current motor level.
 * @param[in] req Request contains the motor level
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool setMotorLevel(pses_ucbridge::SetMotorLevel::Request& req,
                   pses_ucbridge::SetMotorLevel::Response& res,
                   Communication* com)
{
  res.was_set = false;
  std::string cmd = "Drive Forward";
  short level = req.level;
  if (req.level < 0)
  {
    cmd = "Drive Backward";
    level = -req.level;
  }

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("speed", "int16_t", level);
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set Motor Level' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to set the current session id.
 * @param[in] req Request contains the session id
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool setSessionID(pses_ucbridge::SetSessionID::Request& req,
                  pses_ucbridge::SetSessionID::Response& res,
                  Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("ID", "int32_t", req.sid);
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set Session ID' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to change the current steering level.
 * @param[in] req Request contains the steering level
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool setSteeringLevel(pses_ucbridge::SetSteering::Request& req,
                      pses_ucbridge::SetSteering::Response& res,
                      Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("steering", "int16_t", req.steering);
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set Steering Level' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to change the calibration of its ultra sonic range sensors.
 * @param[in] req Request contains range and gain
 * @param[out] res Response message informs the calling process about the
 *success of the service call and adds the new calibration from the us.
 * @returns true is no errors occured, else false.
*/
bool setUS(pses_ucbridge::SetUS::Request& req,
           pses_ucbridge::SetUS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set US";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("usrange", "int16_t", req.range);
  input.insertParameter("usgain", "int16_t", req.gain);
  std::vector<std::string> options;
  options.push_back("US_Range");
  options.push_back("US_Gain");
  options.push_back("US_si_info");
  try
  {
    res.was_set = com->sendCommand(cmd, options, input, output);
    output.getParameterValue("info", res.in_si_units);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Set US' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate its motor brakes.
 * @param[in] req Request contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleBrakes(pses_ucbridge::ToggleBrakes::Request& req,
                  pses_ucbridge::ToggleBrakes::Response& res,
                  Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.brakes_on)
  {
    cmd = "Full Stop";
  }
  else
  {
    cmd = "Drive Forward";
    short level = 0;
    input.insertParameter("speed", "int16_t", level);
  }
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Toggle Brakes' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate the daq.
 * @param[in] req Request contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleDAQ(pses_ucbridge::ToggleDAQ::Request& req,
               pses_ucbridge::ToggleDAQ::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.DAQ_on)
  {
    cmd = "Start DAQ";
  }
  else
  {
    cmd = "Stop DAQ";
  }
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Service 'Toggle DAQ' occured!\n Description: "
                    << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate the dead man's switch.
 * @param[in] req Request contains a bool (true=on, false=off) and the time interval in milliseconds.
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleDMS(pses_ucbridge::ToggleDMS::Request& req,
               pses_ucbridge::ToggleDMS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Drv";
  unsigned int interval = 0;
  if (req.dms_on)
  {
    interval = req.dms_interval;
  }

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::vector<std::string> options;
  options.push_back("DMS_Interval");
  input.insertParameter("dms_time", "uint32_t", interval);
  try
  {
    res.was_set = com->sendCommand(cmd, options, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'toggleDMS' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate a particular group.
 * @param[in] req Request contains a bool (true=on, false=off) and the group number
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleGroup(pses_ucbridge::ToggleGroup::Request& req,
                 pses_ucbridge::ToggleGroup::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.group_on)
  {
    cmd = "Activate Group";
  }
  else
  {
    cmd = "Deactivate Group";
  }
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Toggle Group' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate the kinect power supply.
 * @param[in] req Request contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleKinect(pses_ucbridge::ToggleKinect::Request& req,
                  pses_ucbridge::ToggleKinect::Response& res,
                  Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.kinect_on)
  {
    cmd = "Toggle Kinect On";
  }
  else
  {
    cmd = "Toggle Kinect Off";
  }

  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Toggle Kinect' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate the motor controller.
 * @param[in] req Request contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleMotor(pses_ucbridge::ToggleMotor::Request& req,
                 pses_ucbridge::ToggleMotor::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.motor_on)
  {
    cmd = "Drive Forward";
    short level = 0;
    input.insertParameter("speed", "int16_t", level);
  }
  else
  {
    cmd = "Motor Off";
  }
  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Toggle Motor' occured!\n Description: "
        << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
/**
 * @brief A call to this Service will send a command to the serial device
 * to activate/deactivate the ultra sonic range sensors.
 * @param[in] req Request contains a bool (true=on, false=off)
 * @param[out] res Response message informs the calling process about the
 *success of the service call.
 * @returns true is no errors occured, else false.
*/
bool toggleUS(pses_ucbridge::ToggleUS::Request& req,
              pses_ucbridge::ToggleUS::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if (req.us_on)
  {
    cmd = "Toggle US On";
  }
  else
  {
    cmd = "Toggle US Off";
  }

  try
  {
    res.was_set = com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error in Service 'Toggle US' occured!\n Description: " << e.what());
    res.was_set = false;
    return false;
  }
  return true;
}
}
#endif // SERVICEFUNCTIONS_H
