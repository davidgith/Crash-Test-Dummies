/**
 * @file "pses_ucbridge/Communication/communicationconfig.h"
 * @brief Header file for the CommunicationConfig class.
 *
*/

#ifndef COMMUNICATIONCONFIG_H
#define COMMUNICATIONCONFIG_H

#include <pses_ucbridge/Communication/command.h>
#include <pses_ucbridge/Communication/sensorgroup.h>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <pses_ucbridge/Communication/syntax.h>

/**
 * @struct SerialInterfaceParams communicationconfig.h
 * @brief The SerialInterfaceParams class serves as a data struct to configure
 * the setup of a serial connection.
 *
*/
struct SerialInterfaceParams
{
  unsigned int baudRate; /**< Baudrate of the serial connection (for rs232: baudrate := bit/s) */
  std::string deviceTag; /**< identifier of a serial device */
  unsigned int serialTimeout; /**< How long should the interface wait (ms) for a buffer interrupt? */
  unsigned int maxLineLength; /**< How many characters can be parsed in one query. */
  std::string serialDevicesFolder; /**< Path to the serial device folder of the OS. */
};

/**
 * @class CommunicationConfig communicationconfig.h
 * @brief The CommunicationConfig class parses config files and provides
 *configuration parameters to a Communcation object and creates Command objects
 *as well as SensorGroup objects.
 *
 * To instantiate an object of this class, a valid path to a config folder is
 *required. The folder must contain the following files:
 * channels.yml, command_options.yml, commands.yml, gerneral_syntax.yml,
 *sensor_groups.yml, serial_interface_config.yml
 *
*/
class CommunicationConfig
{
public:
  /**
   * @brief CommunicationConfig default constructor.
  */
  CommunicationConfig();
  /**
   * @brief CommunicationConfig constructor.
   * @param[in] configPath path to the config folder containing all required
   * config files
   * @throws std::exception
  */
  CommunicationConfig(std::string configPath);
  /**
   * @brief Assing operator for YAML::Node to Syntax object.
   * @param[in] node Yaml::Node containing Syntax parameters
   * @param[out] syntax Syntax object to be configured by given parameters
   * @throws std::exception
  */
  friend void operator>>(const YAML::Node& node, Syntax& syntax);
  /**
   * @brief Assing operator for YAML::Node to SerialInterfaceParams object.
   * @param[in] node Yaml::Node containing SerialInterfaceParams parameters
   * @param[out] serialParams SerialInterfaceParams object to be configured by
   * given parameters
   * @throws std::exception
  */
  friend void operator>>(const YAML::Node& node,
                         SerialInterfaceParams& serialParams);
  /**
   * @brief Assing operator for YAML::Node to Channel map object.
   * @param[in] node Yaml::Node containing Channel definitions
   * @param[out] channels map of Channel objects to be configured by given
   * definitions
   * @throws std::exception
  */
  friend void operator>>(
      const YAML::Node& node,
      std::unordered_map<std::string, std::shared_ptr<Channel>>& channels);
  /**
   * @brief Assing operator for YAML::Node to CommandOptions map object.
   * @param[in] node Yaml::Node containing CommandOptions definitions
   * @param[out] options map of CommandOptions objects to be configured by
   * given definitions
   * @throws std::exception
  */
  friend void
  operator>>(const YAML::Node& node,
             std::unordered_map<std::string, std::shared_ptr<CommandOptions>>&
                 options);

  /**
   * @brief Syntax object getter.
   * @return a shared pointer to the Syntax object
  */
  const std::shared_ptr<Syntax> getSyntax() const;
  /**
   * @brief SerialInterfaceParams object getter.
   * @return a shared pointer to the SerialInterfaceParams object
  */
  const std::shared_ptr<SerialInterfaceParams> getSerialInterfaceParams() const;
  /**
   * @brief Command map getter.
   * @return a shared pointer to the Command map object
  */
  const std::unordered_map<std::string, std::shared_ptr<Command>>&
  getCommands() const;
  /**
   * @brief SensorGroup map object getter.
   * @return a shared pointer to the SensorGroup map object
  */
  const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>&
  getSensorGroups() const;

private:
  std::string configPath; /**< Path to configuration folder */
  std::unordered_map<std::string, std::shared_ptr<Command>>
      commands; /**< Command map (first: command name, second: Command) */
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>
      sensorGroups; /**< SensorGroup map (first: group number, second:
                       SensorGroup)*/
  std::unordered_map<std::string, std::shared_ptr<Channel>>
      channels; /**< Channel map (first: channel name, second: Channel)*/
  std::unordered_map<std::string, std::shared_ptr<CommandOptions>>
      options;   /**< CommandOption map (first: option name, second:
                    CommandOptions)*/
  Syntax syntax; /**< Syntax object */
  SerialInterfaceParams serialParams; /**< Serial communication parameter */

  /**
   * @brief Parses Command parameters from a YAML::Node and inserts a new
   * Command object into the Command map.
   * @param[in] node Yaml::Node containing Command definitions
   * @throws std::exception
  */
  void insertCommand(const YAML::Node& node);
  /**
   * @brief Parses SensorGroup parameters from a YAML::Node and inserts a new
   * SensorGroup object into the SensorGroup map.
   * @param[in] node Yaml::Node containing SensorGroup definitions
   * @throws std::exception
  */
  void insertSensorGroup(const YAML::Node& node);
  /**
   * @brief Parses serial interface coniguration from file.
   * @throws std::exception
  */
  void readSerialInterfaceConfig();
  /**
   * @brief Parses Syntax defnition from file.
   * @throws std::exception
  */
  void readGeneralSyntax();
  /**
   * @brief Parses all Channel defnitions from file.
   * @throws std::exception
  */
  void readChannels();
  /**
   * @brief Parses all CommandOptions definitions from file.
   * @throws std::exception
  */
  void readOptions();
  /**
   * @brief Parses all Command definitions from file.
   * @throws std::exception
  */
  void readCommands();
  /**
   * @brief Parses all SensorGroup definitions from file.
   * @throws std::exception
  */
  void readSensorGroups();
};

#endif // COMMUNICATIONCONFIG_H
