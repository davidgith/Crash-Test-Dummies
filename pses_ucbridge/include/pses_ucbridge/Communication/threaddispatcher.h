/**
 * @file "pses_ucbridge/Communication/threaddispatcher.h"
 * @brief Header file for the ThreadDispatcher class.
 *
*/

#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <string>
#include <queue>
#include <pses_ucbridge/Communication/serialinterface.h>
#include <pses_ucbridge/Communication/communicationthread.h>
#include <pses_ucbridge/Communication/readingthread.h>
#include <pses_ucbridge/Communication/communicationconfig.h>
#include <pses_ucbridge/Communication/sensorgroupthread.h>

class ReadingThread;
class SensorGroupThread;

/**
 * @typedef boost::function<void(const std::string&)>
 * @brief Debug callback function pointer.
 * @param[out] debug Contents of the debug message.
*/
typedef boost::function<void(const std::string&)> debugCallbackPtr;

/**
 * @class ThreadDispatcher threaddispatcher.h
 * @brief The ThreadDispatcher class coordinates other communication threads
 * and handles start up and shut down procedures.
 *
*/
class ThreadDispatcher : public CommunicationThread
{
public:
  /**
   * @brief ThreadDispatcher constructor.
   * @param[in] syntax Syntax object
  */
  ThreadDispatcher(const std::shared_ptr<Syntax>& syntax);
  /**
   * @brief Start dispatching thread, SensorGroupThread and ReadingThread
  */
  void startThread();
  /**
   * @brief Stop dispatching thread, SensorGroupThread and ReadingThread
  */
  void stopThread();
  /**
   * @brief Enable pass through of every line in the input buffer.
   *
   * This will call the registered callback whenever there is a new line in
   * the input buffer. The callback decides what to do with that information.
   * @param[in] debug Function pointer of the debug callback.
  */
  void enableDebugMessages(debugCallbackPtr debug);
  /**
   * @brief Enables the transmission of raw messages to the serial device.
  */
  void enableRawCommunication();
  /**
   * @brief Register ReadingThread with the thread dispatcher
   * @param rxThread pointer to a ReadingThread object
  */
  void setReadingThread(ReadingThread* rxThread);
  /**
   * @brief Register SensorGroupThread with the thread dispatcher
   * @param rxThread pointer to a SensorGroupThread object
  */
  void setSensorGroupThread(SensorGroupThread* grpThread);
  /**
   * @brief Register the condition variable of the Communication object in order to wakeup a waiting command response thread
   * @param condVar pointer to a condition variable
  */
  void setCommunicationCondVar(std::condition_variable* condVar);
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
   * @brief Pop a received command response from the response queue.
   * @param[in] response response message from the serial device
  */
  void dequeueResponse(std::string& response);
  /**
   * @brief Pop a received sensor group message from the message queue.
   * @param[in] response sensor group message from the serial device
  */
  void dequeueSensorGroupMessage(std::string& response);
  /**
   * @brief Is there a response in the response queue?
   * @return true if response availible, else false
  */
  const bool IsResponseQueueEmpty() const;
  /**
   * @brief Is there a message in the message queue?
   * @return true if message availible, else false
  */
  const bool IsMessageQueueEmpty() const;
  /**
   * @brief Set wether there is a command response thread waiting or not.
   * @param wakeUp true a command is waiting for response
  */
  void setCommunicationWakeUp(bool wakeUp);

private:

  ReadingThread* readingThread; /**< Pointer to the ReadingThread object */
  SensorGroupThread* sensorGroupThread; /**< Pointer to the SensorGroupThread object */
  debugCallbackPtr debug; /**< Pointer to the debug handling function */
  debugCallbackPtr error; /**< Pointer to the error handling function */
  debugCallbackPtr text; /**< Pointer to the text handling function */
  bool debugMsgEnabled; /**< Is a debug callback available? */
  bool rawCommunicationEnabled; /**< Is a raw communication enabled? */
  bool errorCBregistered; /**< Is an error callback available? */
  bool textCBregistered; /**< Is an text callback available? */
  std::shared_ptr<Syntax> syntax; /**< Pointer to a Syntax object */
  std::queue<std::string> commandResponse; /**< Incoming command response queue */
  std::condition_variable* comCV; /**< condition variable of the Communication object */
  std::queue<std::string> sensorGroupMessage; /**< Incoming sensor message queue */

  /**
   * @brief Worker function, responsible for dispatching the correct thread
   * and observing incoming messages from the serial device.
  */
  void workerFunction();
  bool wakeUpCommunication;
};

#endif // THREADDISPATCHER_H
