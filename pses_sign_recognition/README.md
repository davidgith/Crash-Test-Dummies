# PSES HelloWorld

This ROS-package serves as ... ToDo

### Prerequisites

ROS-Package find_object_2d must be installed.

### Installing

ToDO

## Getting Started

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

## Launching 3D Map Visualisation

Follow instructions from *Getting started*

Start RViz:
`rosrun rviz rviz`

In RViz click *add*, select *TF*, press *ok*.

Set *Fixed Frame* to *kinect2_rgb_optical_frame*.

## Authors

* **Regis Fayard**

