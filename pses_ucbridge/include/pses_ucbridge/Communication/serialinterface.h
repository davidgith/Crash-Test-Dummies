/**
 * @file "pses_ucbridge/Communication/serialinterface.h"
 * @brief Header file for the SerialInterface class.
 *
*/

#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <serial/serial.h>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>

/**
 * @class SerialInterface serialinterface.h
 * @brief The SerialInterface class provides an interface to a connected serial device.
 *
 * This class follows the singleton pattern paradigm.
 * For object instatiation call the instance methond, wich returns a singleton of this class.
 *
*/
class SerialInterface
{
public:
  /**
   * @brief Returns a singleton auf this class.
   * @return static object of SerialInterface class.
  */
  static SerialInterface& instance()
  {
    static SerialInterface _instance;
    return _instance;
  }
  /**
   * @brief SerialInterface default destructor.
  */
  ~SerialInterface() {}
  /**
   * @brief Setter for the baudrate, which defines the bandwith of the communication.
   *
   * For a serial connection using the UART protocol baudrate equals bit/s.
   * @param[in] baudRate in Baud per second
  */
  void setBaudRate(unsigned int baudRate);
  /**
   * @brief Setter for the device tag, which is an identifier a the connected serial device.
   * @param[in] deviceTag identifier of the serial device
  */
  void setDeviceTag(const std::string& deviceTag);
  /**
   * @brief Setter for the serial timeout.
   *
   * How long should the interface wait for activity on the serial connection
   * befor a retry is attempted?
   * @param[in] serialTimeout timeout in milliseconds
  */
  void setSerialTimeout(unsigned int serialTimeout);
  /**
   * @brief Setter for the maximum line length.
   *
   * What is the maximum amount of symbols that can be retrieved from the serial
   * input buffer in one go.
   * @param[in] maxLineLength amount of symbols in a single frame
  */
  void setMaxLineLength(unsigned int maxLineLength);
  /**
   * @brief Setter for the path to the devices folder.
   * @param[in] serialDeviceFolger path where the serial devices are located in an OS
  */
  void setSerialDevicesFolder(const std::string& serialDevicesFolder);
  /**
   * @brief try to open a connection to the serial device
   * @throws std::exception
  */
  void connect();
  /**
   * @brief try to send a message to the serial device
   * @param[in] message message to be transmitted
   * @throws std::exception
  */
  void send(std::string& message);
  /**
   * @brief read a message from the serial input buffer
   * @param[out] message received message
   * @param[in] delimiter stop symbol for the input buffer parser
   * @throws std::exception
  */
  void read(std::string& message, std::string& delimiter);
  /**
   * @brief try to close the connection to the serial device
   * @throws std::exception
  */
  void disconnect();

private:
  // private class attributes
  unsigned int baudRate; /**< Baudrate which defines the bandwith of the communication */
  std::string deviceTag; /**< Device tag which is an identifier a the connected serial device */
  unsigned int serialTimeout; /** <Max interface wait time for activity. */
  unsigned int  maxLineLength; /** <Max amount of symbols in a line. */
  std::string serialDevicesFolder; /** <Path to the serial devices folder. */
  serial::Serial serialConnection; /** <SerialConnection object. */
  // private methods & constructors
  /**
   * @brief SerialInterface default constructor.
  */
  SerialInterface();
  /**
   * @brief SerialInterface default copy constructor.
  */
  SerialInterface(const SerialInterface&);
  /**
   * @brief SerialInterface default assign operator.
  */
  SerialInterface& operator=(const SerialInterface&);

  void findDeviceName(std::string& deviceName);
};

#endif // SERIALINTERFACE_H
