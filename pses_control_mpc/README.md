# PSES Control MPC

This ROS-package serves as a simple example for MPC control by image. 

### Prerequisites

This project was build with ROS Kinetic but should work on older ROS versions as well.
The package is functionally dependent on the pses_ucbridge package and the pses robot.

### Installing

Install eigen3 and any other required libraries.

Build the package with catkin_make:

`catkin_make`

## Getting Started

Launch the communication:

`roslaunch pses_ucbridge uc_bridge`

Run kinect2_bridge:

`rosrun kinect2_bridge kinect2_bridge`

Run the main node of this package using rosrun:

`rosrun pses_control_mpc pses_control_mpc`

Reconfigure using rqt_reconfigure:

`rosrun rqt_reconfigure rqt_reconfigure`

## Authors

* **Kai Cui**
