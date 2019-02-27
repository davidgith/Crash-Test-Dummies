# Road Sign Recognition

This ROS-package uses the kinect image to search for the road signs: *Stop*, *Change lane* and *Speed limit*.  

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

### Topics

The node subscribes to the topic `kinect2/qhd/image_color`

Publishes to `/sign_detection_node/StopSign` if stop sign is detected
Publishes to `/sign_detection_node/LaneSign` if change lane sign is detected
Publishes to `/sign_detection_node/SpeedSign` if speed limit sign is detected

### How to use the find_object ROS-Package

Launch the communication:
`roslaunch pses_ucbridge uc_bridge.launch`

Launch kinect camera:
`roslaunch kinect2_bridge kinect2_bridge.launch`

Launch find_object:
`roslaunch pses_sign_recognition find_sign.launch`

find_object will then use the settings saved in `~/catkin_ws/src/pses_sign_recognition/data/find-sign-settings.ini`
and use the the reference objects saved in 
`~/catkin_ws/src/pses_sign_recognition/data/objs`

(Optional) Print found objects in console.
`rosrun find_object_2d print_objects_detected`

### Launching 3D Map Visualisation for find_object

Follow instructions from *How to use the find_object ROS-Package*

Start RViz:
`rosrun rviz rviz`

In RViz click *add*, select *TF*, press *ok*.

Set *Fixed Frame* to *kinect2_rgb_optical_frame*.

## Authors

* **Regis Fayard**

