/**
 * @file "dashboard_node.cpp"
 * @brief Main node of the pses_dashboard package.
 *
*/

#include <ros/ros.h>
#include <qt5/QtWidgets/qapplication.h>
#include <pses_dashboard/dashboard.h>
#include <signal.h>
#include <unistd.h>

/**
 * @brief This function is the callback for the shutdown system signal.
 *
 * If the OS catches a shutdown system signal (e.g. ctrl+x in command line)
 * this function will be called and shut down the Gui.
 * @param[in] sig os signals
 * @param[in] sig os signals to be ignored
*/
void catchUnixSignals(
    const std::vector<int>& quitSignals,
    const std::vector<int>& ignoreSignals = std::vector<int>())
{

  auto handler = [](int sig) -> void
  {
    printf("\nquit the application (user request signal = %d).\n", sig);
    QCoreApplication::quit();
  };

  // all these signals will be ignored.
  for (int sig : ignoreSignals)
    signal(sig, SIG_IGN);

  // each of these signals calls the handler (quits the QCoreApplication).
  for (int sig : quitSignals)
    signal(sig, handler);
}

/**
 * @brief Main function of this node.
*/
int main(int argc, char** argv)
{

  ros::init(argc, argv, "dashboard");
  ros::NodeHandle nh;
  QApplication a(argc, argv);
  catchUnixSignals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});
  Dashboard w(&nh);
  w.show();
  return a.exec();
}
