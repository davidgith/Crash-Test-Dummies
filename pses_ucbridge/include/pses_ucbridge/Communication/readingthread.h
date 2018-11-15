/**
 * @file "pses_ucbridge/Communication/readingthread.h"
 * @brief Header file for the ReadingThread class.
 *
*/

#ifndef READINGTHREAD_H
#define READINGTHREAD_H

#include <string>
#include <queue>
#include <pses_ucbridge/Communication/communicationconfig.h>
#include <pses_ucbridge/Communication/communicationthread.h>
#include <pses_ucbridge/Communication/threaddispatcher.h>
#include <boost/range/algorithm/remove_if.hpp>

class ThreadDispatcher;

/**
 * @typedef boost::function<void(const std::string&)>
 * @brief Error callback function pointer.
 * @param[out] error Contents of the error message.
*/
typedef boost::function<void(const std::string&)> errorCallbackPtr;

/**
 * @class ReadingThread readingthread.h
 * @brief The ReadingThread class implements threadded polling of the serial input buffer.
 *
*/
class ReadingThread : public CommunicationThread
{
public:
  /**
   * @brief ReadingThread constructor.
   * @param[in] syntax Syntax object
   * @param[in] dispatcher pointer to a ThreadDispatcher object
  */
  ReadingThread(std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher);
  /**
   * @brief Start the polling of the serial input buffer..
  */
  void startThread();
  /**
   * @brief Stop the polling of the serial input buffer..
  */
  void stopThread();
  /**
   * @brief Register an error function pointer to handle errors during communication.
   * @param[in] error this function decides what to do with the contents of an error message
  */
  void registerErrorCallback(errorCallbackPtr error);
  /**
   * @brief Get retrieved data vom the serial input buffer.
   * @return a parsed line from the buffer
  */
  std::string getData();
  /**
   * @brief Is a new line availible?
   * @return true if a line has been retrieved from the buffer, elso false
  */
  const bool isQueueEmpty() const;
private:
  std::shared_ptr<Syntax> syntax; /**< Pointer to a Syntax object */
  std::queue<std::string> data; /**< Queue containing retrieved lines */
  ThreadDispatcher* dispatcher; /**< Dispatcher object managing multi threading */
  errorCallbackPtr error; /**< Pointer to the error handling function */
  bool errorCBregistered; /**< Is an error callback availible? */
  /**
   * @brief Worker function, responsible for polling the serial input buffer.
  */
  void workerFunction();
};

#endif // READINGTHREAD_H
