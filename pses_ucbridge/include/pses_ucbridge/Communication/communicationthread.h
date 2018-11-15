/**
 * @file "pses_ucbridge/Communication/communicationthread.h"
 * @brief Header file for the CommunicationThread class.
 *
*/

#ifndef COMMUNICATIONTHREAD_H
#define COMMUNICATIONTHREAD_H

#include <condition_variable>
#include <mutex>
#include <thread>

/**
 * @class CommunicationThread communicationthread.h
 * @brief The CommunicationThread class is the abstract base class of all thread
 *objects used in the communication module.
 *
 * Every object taking part in the communication module that has threaded
 *functionality should implement this base class.
 * It provides thread safety functionality and makes multithreading manageable
 *and easy to use.
 *
*/
class CommunicationThread
{
public:
  bool active;        /**< Is this thread active? */
  std::thread worker; /**< Generic worker thread of this object. */

  /**
   * @brief CommunicationThread default constructor.
  */
  inline CommunicationThread()
  {
    lock = std::unique_lock<std::mutex>(m);
    notified = false;
    active = false;
  }

  /**
   * @brief CommunicationThread default Move initialization.
   *
   * -> Move initialization is disabled.
  */
  CommunicationThread(CommunicationThread&& other) = delete;

  /**
   * @brief CommunicationThread default Copy initialization.
   *
   * -> Copy initialization is disabled.
  */
  CommunicationThread(const CommunicationThread& other) = delete;

  /**
   * @brief CommunicationThread default Move assignment.
   *
   * -> Move assignment is disabled.
  */
  CommunicationThread& operator=(CommunicationThread&& other) = delete;

  /**
   * @brief CommunicationThread default Copy assignment.
  */
  CommunicationThread& operator=(const CommunicationThread& other)
  {
    std::lock(m, other.m);
    std::lock_guard<std::mutex> self_lock(m, std::adopt_lock);
    std::lock_guard<std::mutex> other_lock(other.m, std::adopt_lock);
    active = other.active;
    notified = other.notified;
    lock = std::unique_lock<std::mutex>(m);
    return *this;
  }

  /**
   * @brief Start the threadded workload.
  */
  virtual void startThread() = 0;
  /**
   * @brief Stop the threadded workload.
  */
  virtual void stopThread() = 0;
  /**
   * @brief Generic threadded worker function of this object.
  */
  virtual void workerFunction() = 0;
  /**
   * @brief Locks the current worker thread until wakeUp() is called.
  */
  inline void sleep()
  {
    notified = false;
    while (!notified) // loop to avoid spurious wakeups
    {
      cond_var.wait(lock);
    }
  }
  /**
   * @brief Unlocks the current worker thread.
  */
  inline void wakeUp()
  {
    notified = true;
    cond_var.notify_one();
  }

private:
  mutable std::mutex m;              /**< Mutex object. */
  std::condition_variable cond_var;  /**< Condition variable. */
  std::unique_lock<std::mutex> lock; /**< Lock object. */
  bool notified;                     /**< Has this thread been notified? */
};

#endif // COMMUNICATIONTHREAD_H
