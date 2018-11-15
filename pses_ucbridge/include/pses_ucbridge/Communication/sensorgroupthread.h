/**
 * @file "pses_ucbridge/Communication/sensorgroupthread.h"
 * @brief Header file for the SensorGroupThread class.
 *
*/

#ifndef SENSORGROUPTHREAD_H
#define SENSORGROUPTHREAD_H

#include <string>
#include <queue>
#include <pses_ucbridge/Communication/communicationthread.h>
#include <pses_ucbridge/Communication/threaddispatcher.h>
#include <pses_ucbridge/Communication/sensorgroup.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>

class ThreadDispatcher;

/**
 * @typedef boost::function<void(const std::string&)>
 * @brief Error callback function pointer.
 * @param[out] error Contents of the error message.
*/
typedef boost::function<void(const std::string&)> errorCallbackPtr;

/**
 * @class SensorGroupThread sensorgroupthread.h
 * @brief The SensorGroupThread class implements functionality to extract the
 *group identifier
 * of an incoming sensor group message and starts the parsing function of the
 *correct SensorGroup object.
 *
*/
class SensorGroupThread : public CommunicationThread
{
public:
  /**
   * @brief SensorGroupThread constructor.
   * @param[in] syntax Syntax object
   * @param[in] dispatcher pointer to a ThreadDispatcher object
   * @param[in] sensorGroups map of SensorGroup object which can be accessed by
   * this class.
  */
  SensorGroupThread(
      std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher,
      const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>&
          sensorGroups);
  /**
   * @brief Start parsing incoming sensor group messages
  */
  void startThread();
  /**
   * @brief Stop parsing incoming sensor group messages
  */
  void stopThread();
  /**
   * @brief Register an error callback that can be called to treat occuing
   * communication and parsing errors.
   * @param error pointer to the error handling callback which decides what to
   * do with an error message.
  */
  void registerErrorCallback(errorCallbackPtr error);

private:
  ThreadDispatcher*
      dispatcher; /**< Dispatcher object managing multi threading */
  std::shared_ptr<Syntax> syntax; /**< Pointer to a Syntax object */
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>
      sensorGroups;       /**< SensorGroup map (first: group number, second:
SensorGroup)*/
  errorCallbackPtr error; /**< Pointer to the error handling function */
  bool errorCBregistered; /**< Is an error callback available? */

  /**
   * @brief Worker function, responsible for parsing sensor messages and calling
   * the correct sensor group processing function.
  */
  void workerFunction();
};

#endif // SENSORGROUPTHREAD_H
