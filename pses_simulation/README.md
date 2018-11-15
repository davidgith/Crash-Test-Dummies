# PSES Simulation

This ROS-package provides a simple simulation for the kinematic model, the ultra sonic range sensors and the kinect of the pses robot from the Real Time Systems lab at FG Echtzeitsysteme - Tu-Darmstadt. The driving simulation based on the Ackermann kinematic model (Cars) and does not account for torque, friction or any forces. Sensory information is simulated by scanning for obstacles on a given 2D grid map at the position of the virtual model of the car.

### Prerequisites

This project was build with ROS Kinetic but should work on older ROS versions as well.
It has a build dependencies on OpenCv3 and can ony be used to simulate the pses robot, otherwise this simulation is useless to you.
### Installing

Clone the repo into your ROS src folder:

`cd ~/catkin_ws/src`

`git clone https://github.com/tud-pses/pses_simulation.git`

`cd ..`

If not previously installed, install the map_server ros package:

`sudo apt-get install ros-kinect-map-server`

Build the package with catkin_make:

`catkin_make`

## Getting Started

Please feel free to browse our wiki, where you can find instructions on how to use and configure this package.

[Pses_Simulation Wiki](https://github.com/tud-pses/pses_simulation/wiki)

If you're looking for a documentation of the code, please follow this link:

[C++ documentation](https://tud-pses.github.io/pses_simulation/)

## Authors

* **Nicolas Acero**
* **Sebastian Ehmes**
