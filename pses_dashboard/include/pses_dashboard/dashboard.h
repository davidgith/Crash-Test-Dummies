/**
 * @file "pses_dashboard/dashboard.h"
 * @brief Header file for the Dashboard class.
 *
*/

#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <math.h>
#include <string>
#include <sstream>
#include <boost/math/special_functions/sign.hpp>

#include <qt5/QtWidgets/QMainWindow>
#include <qt5/QtCore/qtimer.h>
#include <qt5/QtGui/QKeyEvent>
#include <ui_dashboard.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <pses_ucbridge/GetFirmwareVersion.h>
#include <pses_ucbridge/GetControllerID.h>
#include <pses_ucbridge/GetSessionID.h>
#include <pses_ucbridge/ToggleKinect.h>
#include <pses_ucbridge/ToggleMotor.h>
#include <pses_ucbridge/ToggleUS.h>
#include <pses_ucbridge/ToggleDAQ.h>

/**
 * @namespace Dash
 * @brief This namespace contains all typedefs and constants used in the
 *Dashboard class.
 *
*/
namespace Dash
{
/**
 * @typedef std_msgs::String string_msg
 * @brief shortcut for standard String messages
*/
typedef std_msgs::String string_msg;
/**
 * @typedef sensor_msgs::Image image_msg
 * @brief shortcut for Image messages
*/
typedef sensor_msgs::Image image_msg;
/**
 * @typedef sensor_msgs::Imu imu_msg
 * @brief shortcut for IMU messages
*/
typedef sensor_msgs::Imu imu_msg;
/**
 * @typedef sensor_msgs::BatteryState battery_msg
 * @brief shortcut for battery messages
*/
typedef sensor_msgs::BatteryState battery_msg;
/**
 * @typedef sensor_msgs::MagneticField magnetic_msg
 * @brief shortcut for Magnetic Field messages
*/
typedef sensor_msgs::MagneticField magnetic_msg;
/**
 * @typedef sensor_msgs::Range range_msg
 * @brief shortcut for Range messages
*/
typedef sensor_msgs::Range range_msg;
/**
 * @typedef std_msgs::Int16 int16_msg
 * @brief shortcut for int16 messages
*/
typedef std_msgs::Int16 int16_msg;
/**
 * @typedef std_msgs::Float64 float64_msg
 * @brief shortcut for double messages
*/
typedef std_msgs::Float64 float64_msg;
/**
 * @typedef std_msgs::UInt8 uint8_msg
 * @brief shortcut for uint8 messages
*/
typedef std_msgs::UInt8 uint8_msg;
/**
 * @typedef nav_msgs::Odometry odom_msg
 * @brief shortcut for Odometry messages
*/
typedef nav_msgs::Odometry odom_msg;

static const std::string DEFAULT_MODE_CONTROL_TOPIC =
    "/dasboard/mode_control"; /**< Default topic on which mode selection
                                 ispublished. */
static const std::string DEFAULT_STEERING_COMMAND_TOPIC =
    "/uc_bridge/set_steering_level_msg"; /**< Default topic on which steering
                                            level is published. */
static const std::string DEFAULT_MOTOR_COMMAND_TOPIC =
    "/uc_bridge/set_motor_level_msg"; /**< Default topic on which motor
                                         level is published. */
static const std::string DEFAULT_IMU_TOPIC =
    "/uc_bridge/imu"; /**< Default topic from which IMU messages can be
                         received. */
static const std::string DEFAULT_MAGNETIC_TOPIC =
    "/uc_bridge/mag"; /**< Default topic from which
                         Magenetic Field messages can
                         be received. */
static const std::string DEFAULT_USR_TOPIC =
    "/uc_bridge/usr"; /**< Default topic from which right ultra sonic range
                         messages can be received. */
static const std::string DEFAULT_USF_TOPIC =
    "/uc_bridge/usf"; /**< Default topic from which front ultra sonic range
                         messages can be received. */
static const std::string DEFAULT_USL_TOPIC =
    "/uc_bridge/usl"; /**< Default topic from which left ultra sonic range
                         messages can be received. */
static const std::string DEFAULT_HALLCNT_TOPIC =
    "/uc_bridge/hall_cnt"; /**< Default topic from which hall count
                              messages can be received. */
static const std::string DEFAULT_HALLDT_TOPIC =
    "/uc_bridge/hall_dt"; /**< Default topic from which hall time interval
                             messages can be received. */
static const std::string DEFAULT_HALLDT8_TOPIC =
    "/uc_bridge/hall_dt8"; /**< Default topic from which hall full rotation
                              interval messages can be received. */
static const std::string DEFAULT_VDBAT_TOPIC = "/uc_bridge/vdbat"; /**< Default
topic from which drive battery messages can be received. */
static const std::string DEFAULT_VSBAT_TOPIC =
    "/uc_bridge/vsbat"; /**< Default topic from which system battery messages
                           can be received. */
static const std::string DEFAULT_ODOM_TOPIC =
    "/odom"; /**< Default topic from which odometry messages can be received. */
static const std::string DEFAULT_GET_FIRMWARE_SERVICE =
    "/uc_bridge/get_firmware_version"; /**< Default service from which the
                                          ucboard firmware version can be
                                          retreived. */
static const std::string DEFAULT_GET_CARID_SERVICE =
    "/uc_bridge/get_controller_id"; /**< Default service from which the
                                       ucboard id can be retreived. */
static const std::string DEFAULT_GET_SID_SERVICE =
    "/uc_bridge/get_session_id"; /**< Default service from which the ucboard
                                    session id can be retreived. */
static const std::string DEFAULT_IMAGE_COLOR_TOPIC =
    "kinect2/qhd/image_color"; /**< Default topic from which qhd color images
                                  can be received. */
static const std::string DEFAULT_IMAGE_DEPTH_TOPIC =
    "kinect2/sd/image_depth"; /**< Default topic from which sd depth
                                 images can be received. */
static const std::string DEFAULT_TOGGLE_KINECT_SERVICE =
    "/uc_bridge/toggle_kinect"; /**< Default service that toggles the kinect
                                   power supply. */
static const std::string DEFAULT_TOGGLE_MOTOR_SERVICE =
    "/uc_bridge/toggle_motor"; /**< Default service that toggles the drive
                                  controller. */
static const std::string DEFAULT_TOGGLE_US_SERVICE =
    "/uc_bridge/toggle_us"; /**< Default service that toggles the ultra sonic
                               range sensors. */
static const std::string DEFAULT_TOGGLE_DAQ_SERVICE =
    "/uc_bridge/toggle_daq"; /**< Default service that toggles the daq. */
static const std::string DEFAULT_VIDEO_FEED_MODE =
    "Off"; /**< Default video feed mode on startup. */
static const int DEFAULT_MAX_FWD_SPEED =
    1000; /**< Maximum allowed forward motor level. */
static const int DEFAULT_MAX_BWD_SPEED =
    -500; /**< Maximum allowed backward motor level. */
static const int DEFAULT_SPEED_STEP = 50; /**< Increments in which the motor
                                             level can be changed, either
                                             through the slider widget or key
                                             strokes. */
static const int DEFAULT_MAX_LEFT_STEER =
    -1000; /**< Maximum allowed left steering level. */
static const int DEFAULT_MAX_RIGHT_STEER =
    1000; /**< Maximum allowed right steering level. */
static const int DEFAULT_STEER_STEP = 100; /**< Increments in which the
steering level can be changed, either through the slider widget or key strokes.
*/
static const std::string VIDEO_FEED_MODE_COLOR =
    "Color Image (1280x720)"; /**< Color video feed mode. */
static const std::string VIDEO_FEED_MODE_DEPTH =
    "Depth Image (640x480)"; /**< Depth video feed mode. */
}

using namespace Dash;

/**
 * @namespace Ui
 * @brief This namespace contains the autogenerated ui header.
 *
*/
namespace Ui
{
class Dashboard;
}

/**
 * @class Dashboard dashboard.h "pses_dashboard/dashboard.h"
 * @brief The Dashboard class provides functionality for the autogenerated
 * ui header.
 *
 * This class implements the means for the GUI to interface ROS functionality,
 * i.e. publishing and subscribing to topics as well as call ROS services.
 * The Dashboard GUI is meant to be used in combination with the pses_ucbridge
 *or
 * the pses_simulation package, since its build around the pses robot
 *functionality.
 *
*/
class Dashboard : public QMainWindow
{
  Q_OBJECT

public:
  /**
   * @brief Dashboard constructor.
   * @param[in] nh ros::NodeHandle object needed to interact with the ros
   * framework.
   * @param[in] parent optional parameter to signal if this is a sub widget of
   * another ui container.
  */
  explicit Dashboard(ros::NodeHandle* nh, QWidget* parent = 0);
  /**
   * @brief Dashboard default destructor.
  */
  virtual ~Dashboard();
  /**
   * @brief Callback function in case of a key event.
   *
   * This function decides what to do in case of a pressed key event.
   * @param[in] event signals what key has been pressed
  */
  void keyPressEvent(QKeyEvent* event);

private:
  /**
   * @brief Get all available launch parameters.
  */
  void fetchStartupParameters();
  /**
   * @brief Reconfigure the gui (e.g. speed slider) according to launch
   * paramters.
  */
  void reconfigureSpeedSlider();
  /**
   * @brief Register all published/subscribed ros topics.
  */
  void registerRosTopics();
  /**
   * @brief Get the firmware version from the ucboard.
  */
  void callGetFirmwareServide();
  /**
   * @brief Get the id from the ucboard.
  */
  void callGetCarIdServide();
  /**
   * @brief Get the session id from the ucboard.
  */
  void callGetSidServide();
  /**
   * @brief Set up video feed based on launch parameters.
  */
  void configureVideoFeed();
  /**
   * @brief Register gui relevant callbacks e.g. buttons, silders etc.
  */
  void connectGuiSignals();
  /**
   * @brief Ininit ros topic polling timer.
   *
   * This is needed in case of prolonged inactivity from topics.
  */
  void initTopicPollingTimer();
  /**
   * @brief This function gets called whenever a message has been published to
   *the imu topic.
   *
   * It decides how to display the revceived data.
  */
  void imuCallback(const imu_msg::ConstPtr& imu);
  /**
   * @brief This function gets called whenever a message has been published to
   *the magnetic field topic.
   *
   * It decides how to display the revceived data.
  */
  void magneticCallback(const magnetic_msg::ConstPtr& magnetic);
  /**
   * @brief This function gets called whenever a message has been published to
   *the right ultra sonic range topic.
   *
   * It decides how to display the revceived data.
  */
  void usrCallback(const range_msg::ConstPtr& usr);
  /**
   * @brief This function gets called whenever a message has been published to
   *the left ultra sonic range topic.
   *
   * It decides how to display the revceived data.
  */
  void uslCallback(const range_msg::ConstPtr& usl);
  /**
   * @brief This function gets called whenever a message has been published to
   *the front ultra sonic range topic.
   *
   * It decides how to display the revceived data.
  */
  void usfCallback(const range_msg::ConstPtr& usf);
  /**
   * @brief This function gets called whenever a message has been published to
   *the hall count topic.
   *
   * It decides how to display the revceived data.
  */
  void hallCntCallback(const uint8_msg::ConstPtr& hallCnt);
  /**
   * @brief This function gets called whenever a message has been published to
   *the hall dt topic.
   *
   * It decides how to display the revceived data.
  */
  void hallDtCallback(const float64_msg::ConstPtr& hallDt);
  /**
   * @brief This function gets called whenever a message has been published to
   *the hall dt 8 topic.
   *
   * It decides how to display the revceived data.
  */
  void hallDt8Callback(const float64_msg::ConstPtr& hallDt8);
  /**
   * @brief This function gets called whenever a message has been published to
   *the drive battery topic.
   *
   * It decides how to display the revceived data.
  */
  void driveBatteryCallback(const battery_msg::ConstPtr& vdBat);
  /**
   * @brief This function gets called whenever a message has been published to
   *the system battery topic.
   *
   * It decides how to display the revceived data.
  */
  void systemBatteryCallback(const battery_msg::ConstPtr& vsBat);
  /**
   * @brief This function gets called whenever a message has been published to
   *the color image topic.
   *
   * It decides how to display the revceived data.
  */
  void cameraCallback(const image_msg::ConstPtr& img);
  /**
   * @brief This function gets called whenever a message has been published to
   *the depth image topic.
   *
   * It decides how to display the revceived data.
  */
  void depthCallback(const image_msg::ConstPtr& img);
  /**
   * @brief This function gets called whenever a message has been published to
   *the odometry topic.
   *
   * It decides how to display the revceived data.
  */
  void odomCallback(const odom_msg::ConstPtr& odom);

  Ui::Dashboard* ui;   /**< Pointer to the auto generated ui implementation.*/
  ros::NodeHandle* nh; /**< Pointer to a ros::NodeHandle object.*/
  std::string modeControlTopic, steeringCommandTopic, motorCommandTopic,
      imuTopic, magneticTopic, usrTopic, uslTopic, usfTopic, hallCntTopic,
      hallDtTopic, hallDt8Topic, vdBatTopic, vsBatTopic, getFirmwareService,
      getCarIdService, imageColorTopic, imageDepthTopic, toggleKinectService,
      getSidService, toggleUSService, toggleMotorService, toggleDAQService,
      odomTopic; /**< Subscribed/Published topic names.*/
  int maxForwardSpeed, maxReverseSpeed, speedStep, maxLeftSteering,
      maxRightSteering, steeringStep; /**< Gui parameter*/
  int leftSgn, rightSgn, fwdSgn,
      bwdSgn; /**< The sign of a certain steering/driving direction.*/
  std::string videoFeedMode; /**< Selected video feed mode*/
  int16_msg motorMessage;    /**< Current motor level*/
  int16_msg steeringMessage; /**< Current steering level*/
  string_msg mode;           /**< Current driving mode*/
  double distanceTravelled;  /**< Current travelled distance (only if odometry
                                available)*/
  double x, y, z; /**< Current position (only if odometry available)*/

  ros::Publisher modeControl, motorCommand,
      steeringCommand; /**< ros::Publisher objects.*/
  ros::Subscriber odomSub, imuSub, magneticSub, usrSub, uslSub, usfSub,
      hallCntSub, hallDtSub, hallDt8Sub, vdBatSub, vsBatSub, cameraSub,
      depthSub;  /**< ros::Subscriber objects.*/
  QTimer* timer; /**< Ros queue polling timer.*/

private slots:
  /**
   * @brief This function gets called whenever the kinect toggle button has been
   *pressed.
   *
   * It calls the ros::Service to toggle the kinect power supply.
  */
  void toggleKinect();
  /**
   * @brief This function gets called whenever the us toggle button has been
   *pressed.
   *
   * It calls the ros::Service to toggle the ultra sonic range sensors.
  */
  void toggleUS();
  /**
   * @brief This function gets called whenever the motor toggle button has been
   *pressed.
   *
   * It calls the ros::Service to toggle the motor control.
  */
  void toggleMotor();
  /**
   * @brief This function gets called whenever the daq toggle button has been
   *pressed.
   *
   * It calls the ros::Service to toggle the daq.
  */
  void toggleDAQ();
  /**
   * @brief This function gets called whenever a mode has been selected.
   *
   * It published the selected mode on a mode selection topic.
  */
  void modeSelect(int index);
  /**
   * @brief This function gets called whenever the camera mode changed.
   *
   * It subscribes to the selected camera topic and displays its output.
  */
  void cameraSelect(int index);
  /**
   * @brief This function gets called whenever the speed slider changed.
   *
   * It published the selected speed on the motor level topic.
  */
  void valueChangedSpeed(int value);
  /**
   * @brief This function gets called whenever the steering slider changed.
   *
   * It published the selected speed on the steering level topic.
  */
  void valueChangedSteering(int value);
  /**
   * @brief This function gets called whenever the max speed button has been
   *clicked.
   *
   * It published the max forward speed on the motor level topic.
  */
  void maxSpeedClicked();
  /**
   * @brief This function gets called whenever the min speed button has been
   *clicked.
   *
   * It published the max reverse speed on the motor level topic.
  */
  void minSpeedClicked();
  /**
   * @brief This function gets called whenever the zero speed button has been
   *clicked.
   *
   * It published zero on the motor level topic.
  */
  void zeroSpeedClicked();
  /**
   * @brief This function gets called whenever the max right steering button has
   *been clicked.
   *
   * It published the max right steering on the steering level topic.
  */
  void maxRightSteeringClicked();
  /**
   * @brief This function gets called whenever the max left steering button has
   *been clicked.
   *
   * It published the max left steering on the steering level topic.
  */
  void maxLeftSteeringClicked();
  /**
   * @brief This function gets called whenever the center steering button has
   *been clicked.
   *
   * It published zero on the steering level topic.
  */
  void centerSteeringClicked();
  /**
   * @brief This function gets called whenever the polling timer runs out.
   *
   * It polls the queues of subscribed/published ros topics.
  */
  void pollNodeHandle();
};

#endif // DASHBOARD_H
