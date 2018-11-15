/**
 * @file "pses_ucbridge/Communication/communication.h"
 * @brief Header file for the Communication class.
 *
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_ucbridge/Communication/threaddispatcher.h>
#include <pses_ucbridge/Communication/serialinterface.h>
#include <pses_ucbridge/Communication/communicationconfig.h>
#include <pses_ucbridge/Communication/readingthread.h>
#include <ros/ros.h>
#include <pses_ucbridge/Communication/parameter.h>
#include <pses_ucbridge/Communication/command.h>

/**
 * @class Communication communication.h
 * @brief The Communication class provides access to microcontroller
 *communication.
 *
 * An instance of this Class is needed to gain access to a connected serial
 *device (e.g. microcontroller).
 * Member functions of this class are used to open and close a connection, as
 *well as sending commands
 * and receiving responses and sensory data.
 *
 * To instantiate an object of this class, a valid path to a config folder is
 *required. The folder must contain the following files:
 * channels.yml, command_options.yml, commands.yml, gerneral_syntax.yml,
 *sensor_groups.yml, serial_interface_config.yml
 *
*/
class Communication
{
public:
  /**
   * @brief Communication constructor.
   * @param[in] configPath path to the config folder containing all required config files
   * @throws std::exception
  */
  Communication(const std::string& configPath);
  /**
   * @brief Communication default destructor.
  */
  ~Communication();
  /**
   * @brief Tries to setup a connection to a serial devive with a certain device-tag and specified baudrate.
   * @throws std::exception
  */
  void connect();
  /**
   * @brief Start reading the serial input buffer and start parsing threads.
   * @throws std::exception
  */
  void startCommunication();
  /**
   * @brief Stop reading the serial input buffer and shutdown parsing threads.
   * @throws std::exception
  */
  void stopCommunication();
  /**
   * @brief Terminates a connection to a serial devive with a certain device-tag.
   * @throws std::exception
  */
  void disconnect();
  /**
   * @brief Method to send a string directly to the serial device.
   *
   * This will send the input string as is. If there is some kind of end-of-message-symbol
   * required from the serial device, it will not be appended automatically.
   * @param[in] msg Input string to be transmitted
   * @throws std::exception
  */
  void sendRawMessage(const std::string& msg);
  /**
   * @brief Enable pass through of every line in the input buffer.
   *
   * This will call the registered callback whenever there is a new line in
   * the input buffer. The callback decides what to do with that information.
   * @param[in] debug Function pointer of the debug callback.
   * @throws std::exception
  */
  void enableDebugMessages(debugCallbackPtr debug);
  /**
   * @brief Enables the transmission of raw messages to the serial device.
   * @throws std::exception
  */
  void enableRawCommunication();
  /**
   * @brief Send a command to the servial device.
   *
   * Use this method to send a predefined command to the serial device.
   * Parameters required in the command message need to be provided in the
   * inputParams ParameterMap.
   * Returned parameters from the response will be put in the provided
   * outputParams ParameterMap.
   * @param[in] command unique identifier of the command
   * @param[in] inputParams provided input parameter values
   * @param[out] outputParams returned parameter values
   * @param[in] timeout how long to wait for a response in micro seconds (Default: 100000)
   * @throws std::exception
  */
  bool sendCommand(const std::string& command,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 100000);
  /**
   * @brief Send a command to the servial device with additional options.
   *
   * Use this method to send a predefined command to the serial device.
   * By providing a list of unique command option identifiers this command
   * may be modified according to these options.
   * Parameters required in the command message need to be provided in the
   * inputParams ParameterMap.
   * Returned parameters from the response will be put in the provided
   * outputParams ParameterMap.
   * @param[in] command unique identifier of the command
   * @param[in] options list of unique command option identifiers
   * @param[in] inputParams provided input parameter values
   * @param[out] outputParams returned parameter values
   * @param[in] timeout how long to wait for a response in micro seconds (Default: 100000)
   * @throws std::exception
  */
  bool sendCommand(const std::string& command,
                   const std::vector<std::string>& options,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 100000);
  /**
   * @brief Register a callback to handle transmission/communication errors.
   *
   * The registered function will be called whenever an error during transmission
   * of an outgoing, or parsing of an incoming message occurs.
   * The callback decides what to do with that information.
   * @param[in] error Function pointer of the error callback.
   * @throws std::exception
  */
  void registerErrorCallback(debugCallbackPtr error);
  /**
   * @brief Register a callback to handle plain text messages from the serial device.
   *
   * The registered function will be called whenever a plain text message has been
   * received from the serial device.
   * The callback decides what to do with that information.
   * @param[in] text Function pointer of the text callback.
   * @throws std::exception
  */
  void registerTextCallback(debugCallbackPtr text);
  /**
   * @brief Register all predefined sensor groups on the serial device.
   *
   * @param[in] cmdName unique identifier of the command that provides the
   * functionality to create sensor groups on the serial device.
   * @param[in] timeout how long to wait for a response in micro seconds (Default: 100000)
   * @throws std::exception
  */
  bool registerSensorGroups(const std::string& cmdName,
                            unsigned int timeout = 100000);
  /**
   * @brief Register a callback to handle sensor group messages from the serial device.
   *
   * The registered function will be called whenever a sensor message with the registred
   * group number has been received from the serial device.
   * The callback decides what to do with that information.
   * @param[in] grpNumber The group number is the unique identifier of a sensor group.
   * @param[in] cbPtr Function pointer of the sensor group callback.
   * @throws std::exception
  */
  void registerSensorGroupCallback(const unsigned char& grpNumber,
                                   responseCallback cbPtr);

private:
  CommunicationConfig comCfg; /**< Object that stores all given configuration parameters. */
  std::shared_ptr<SerialInterfaceParams> serialParams; /**< Object that stores serial connection parameters. */
  std::shared_ptr<Syntax> syntax; /**< Object that stores syntax parameters. */
  std::unordered_map<std::string, std::shared_ptr<Command>> commands; /**< List of predefined Command objects */
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>> sensorGroups; /**< List of predefined SensorGroup objects */
  ThreadDispatcher* dispatcher; /**< Thread object that manages multi threaded functionality. */
  bool rawCommunicationEnabled; /**< is direct communication with the serial device enabled? */
  ReadingThread* rxPolling; /**< Thread object which polls the serial input buffer. */
  SensorGroupThread* sensorGroupThread; /**< Thread object which distributes sensor group messages. */
  mutable std::mutex mtx; /**< Mutex object. */
  std::condition_variable cv; /**< Condition variable. */

  /**
   * @brief Configures the serial connection setup with the provided parameters.
   * @throws std::exception
  */
  void configSerialInterface();
};

#endif // COMMUNICATION_H
