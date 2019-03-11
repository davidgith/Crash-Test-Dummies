# Road Sign Recognition

This ROS-package uses the kinect image to search for the road signs: *Stop*, *Change lane* and *Speed limit*.  
The Code is based on the OpenCV Example "SURF_FLANN_matching_homography_Demo.cpp" and has been ajusted to work with ROS, the kinect camera and multi-object detection has been added.


![Stop Sign Image](https://github.com/davidgith/Crash-Test-Dummies/raw/master/pses_sign_recognition/data/objs/stop.png) ![Change lane image](https://github.com/davidgith/Crash-Test-Dummies/raw/master/pses_sign_recognition/data/objs/change_lane.png) ![Speed Limit Sign Image](https://github.com/davidgith/Crash-Test-Dummies/raw/master/pses_sign_recognition/data/objs/50.png)

### Prerequisites

OpenCV3 with the [opencv_contrib modules xfeatures2d](https://github.com/Itseez/opencv_contrib/tree/master/modules/xfeatures2d) must be installed!

This project was build with ROS Kinetic but should work on older ROS versions as well.
The package is functionally dependent on the pses_ucbridge package, the iai_kinect2 package and the pses robot.

### Installing

The correct OpenCV version with the xfeatures2d module can be installed from scouce with the script `installOpenCV3_4.sh` *(Author: Nicolas Acero)*

Build the package with catkin_make:

`catkin_make`

## Getting started
Launch the communication:
`roslaunch pses_ucbridge uc_bridge.launch`

Launch kinect camera:
`roslaunch kinect2_bridge kinect2_bridge.launch`

Run sign detection:
`rosrun pses_sign_recognition obj_detection _gui:=true`
The GUI can be turned on and off with the ROS-Parameter `_gui:=true` or `_gui:=false`

![GUI exampe for Stop Sign Recognition](https://github.com/davidgith/Crash-Test-Dummies/raw/master/pses_sign_recognition/data/gui_example.png)


### Topics

The node subscribes to the topic `kinect2/qhd/image_color`

Publishes distance in cm to `/sign_detection_node/StopSign` if stop sign is detected
Publishes distance in cm to `/sign_detection_node/LaneSign` if change lane sign is detected
Publishes distance in cm to `/sign_detection_node/SpeedSign` if speed limit sign is detected


## Authors

* **Regis Fayard**

