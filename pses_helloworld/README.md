# PSES HelloWorld

This ROS-package serves as a simple example how to subscribe to sensor topics and how to controll the robot through command messages.

### Prerequisites

This project was build with ROS Kinetic but should work on older ROS versions as well.
The package is functionally dependent on the pses_ucbridge package and the pses robot.

### Installing

Clone the repo into your ROS src folder:

`cd ~/catkin_ws/src`

`git clone https://github.com/tud-pses/pses_helloworld.git`

`cd ..`

Build the package with catkin_make:

`catkin_make`

## Getting Started

Launch the communication:

`roslaunch pses_ucbridge uc_bridge`

Run the main node of this package

`rosrun pses_helloworld helloworld_node`

## Authors

* **Nicolas Acero**
* **Sebastian Ehmes**
