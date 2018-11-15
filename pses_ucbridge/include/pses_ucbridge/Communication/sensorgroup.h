/**
 * @file "pses_ucbridge/Communication/sensorgroup.h"
 * @brief Header file for the SensorGroup class.
 *
*/

#ifndef SENSORGROUP_H
#define SENSORGROUP_H

#include <vector>
#include <string>
#include <pses_ucbridge/Communication/parameter.h>
#include <pses_ucbridge/Communication/command.h>
#include <pses_ucbridge/Communication/syntax.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <pses_ucbridge/Communication/base64Decoder.h>

/**
 * @struct Channel sensorgroup.h
 * @brief The Channel class serves as a data struct to store sensor channel
 *parameter.
 *
*/
struct Channel
{
  std::string chName;   /**< Unique identifier of a sensor channel. */
  std::string dataType; /**< Datatype of the sensor channel output */
  bool
      conversionNeeded; /**< Can this channel value be converted to SI units? */
  double conversionFactor; /**< Conversion factor to SI units */
};

/**
 * @struct SensorGroupParameter sensorgroup.h
 * @brief The SensorGroupParameter class serves as a data struct to configure
 * SensorGroup objects.
 *
 * An instance of this struct is required to instantiate
 * a SensorGroup object.
 *
*/
struct SensorGroupParameter
{
  std::string grpName; /**< (optional) semantic descriptor of a sensor group */
  unsigned char grpNumber; /**< Unique identifier of a sensor group. */
  std::vector<std::shared_ptr<Channel>>
      channels; /**< List of sensor channels in a group */
  std::vector<std::pair<std::shared_ptr<CommandOptions>, std::string>>
      options; /**< List of CommandOptions for this group (first: option name,
                  second: option parameter values) */
  std::string encoding; /**< Encoding of the sensor channel data */
};

class SensorGroup;

/**
 * @typedef boost::function<void(SensorGroup*)>
 * @brief SensorGroup callback pointer used to process incoming sensor group
 * data.
 * @param[out] SensorGroup_ptr Pointer to the SensorGroup object that has the
 * received data.
*/
typedef boost::function<void(SensorGroup*)> responseCallback;
/**
 * @typedef boost::function<void(const std::string&)>
 * @brief Value error callback function pointer.
 * @param[out] error Contents of the faulty value error message.
*/
typedef boost::function<void(const std::string&)> valueErrorCallbackPtr;

/**
 * @class SensorGroup sensorgroup.h
 * @brief The SensorGroup class builds the syntactic template for its specific
 * command that results from the sensor group definition and parses incoming
 *sensor group data.
 *
 * A SensorGroup object builds its command message given the required channels
 *and channel options.
 * It can also parse incoming sensor group data and call a registered function
 * which decides what to do with the received data.
*/
class SensorGroup
{
public:
  /**
   * @brief SensorGroup default constructor.
  */
  SensorGroup();
  /**
   * @brief SensorGroup constructor.
   * @param[in] sensorParams serves as a data struct to configure
   * SensorGroup a object.
   * @param[in] syntax Syntax object used to detect escape sequences and other
   * important symbols.
  */
  SensorGroup(const SensorGroupParameter& sensorParams,
              std::shared_ptr<Syntax> syntax);
  /**
   * @brief Parses an incoming sensor group message, extracts contained values and calls the registered sensor group callback.
   * @param[in] response input sensor group message
  */
  void processResponse(const std::string& response);
  /**
   * @brief Register a sensor group callback.
   * @throws std::exception
   * @param[in] callbackFunction pointer to the function that gets call whenever this sensor group object receives a message.
  */
  void setResponseCallback(responseCallback callbackFunction);
  /**
   * @brief Register a value error function pointer to handle value errors during communication.
   * @param[in] valueError this function decides what to do with the contents of a value error message
  */
  void registerErrorCallback(valueErrorCallbackPtr valueError);
  /**
   * @brief Get the name of this sensor group
   * @return name of this sensor group
  */
  const std::string& getName() const;
  /**
   * @brief Get the generated sensor group command, that can be used to register this sensor group on the serial device.
   * @param[in] cmd Command object containing a set sensor group command
   * @param[out] command constructed set sensor group command
  */
  void createSensorGroupCommand(Command& cmd, std::string& command) const;
  /**
   * @brief Verfy the response on a constructed set sensor group command.
   * @param[in] cmd Command object containing a set sensor group command
   * @param[in] response response from the serial device
   * @throws std::exception
   * @return true if response is valid, else false
  */
  const bool verifyResponseOnComand(Command& cmd,
                                    const std::string& response) const;

  /**
   * @brief Get a received value from a certain channel.
   * @param[in] name sensor channel name
   * @param[out] returned (unconverted) value
   * @throws std::exception
   * @return true if channel is in sensor group, else false
  */
  template <typename T>
  inline const bool getChannelValue(const std::string& name, T& value) const
  {
    if (!channelValues.isParamInMap(name))
      return false;
    channelValues.getParameterValue(name, value);
    return true;
  }
  /**
   * @brief Get a received converted value from a certain channel.
   * @param[in] name sensor channel name
   * @param[out] returned converted value
   * @throws std::exception
   * @return true if channel is in sensor group, else false
  */
  const bool getChannelValueConverted(const std::string& name,
                                      double& value) const;

  static const std::string ENCODING_ASCII; /**< Constant Ascii encoding indicator */
  static const std::string ENCODING_B64; /**< Constant b64 encoding indicator */
  static const std::string ENCODING_HEX; /**< Constant hex encoding indicator */

private:
  unsigned char grpNumber; /**< The unique identifier of a group */
  std::string grpName; /**< Optional semanitc descriptor of a group */
  std::string responseEncoding; /**< Encoding standard of a sensor group message */
  std::vector<std::shared_ptr<Channel>> channelList; /**< List of all channels in a sensor group. */
  std::unordered_map<std::string, std::shared_ptr<Channel>> channelMap; /**< Map of Channel objects in this group (first: name, second Channel) */
  Parameter::ParameterMap channelValues; /**< Parameter map of received channel values */
  std::vector<std::string> optionVariableList; /**< List of parameter names for given options */
  Parameter::ParameterMap optionValues; /**< Parameter map of given option values */
  responseCallback callbackFunction; /**< Sensor group message callback pointer */
  bool callbackRegistered; /**< Has a group callback been registered? */
  Parameter::ParameterMap cmdInputParams; /**< Set group command input parameter map */
  std::vector<std::string> optionsList; /**< List of enabled options for a group */
  std::shared_ptr<Syntax> syntax; /**< Pointer to a Syntax object */
  valueErrorCallbackPtr valueError; /**< Pointer to the value error handling function */
  bool valueErrorCBSet; /**< Has a value error callback been registered? */

  /**
   * @brief Parse a received sensor channel message and extract channel values, i.e. convert encoded data to "proper" data types
   * @param[in] response posted sensor group message
   * @throws std::exception
  */
  void parseResponse(const std::string& response);
};

#endif // SENSORGROUP_H
