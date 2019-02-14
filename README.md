# Crash-Test-Dummies
This is the PSES implementation repository of the group 'Crash-Test-Dummies'. It implements algorithms to follow colored lanes and react to traffic signals.

## Getting Started

Please feel free to browse the project's documentation, where you can find instructions on how to use and configure this package.

[Pses_docs](https://github.com/tud-pses/pses_docs)

For a C++ documentation, you can see

[C++ Documentation](https://github.com/tud-pses/pses_docs)

For OpenCV 3.4, you can use the script installOpenCV3_4.sh. 

After installing required dependencies, you can start the control node and begin driving. To give the control nodes commands, you can use dynamic reconfiguration, i.e. rqt_reconfigure. For example: 

```
roslaunch pses_ucbridge uc_bridge.launch 
roslaunch pses_dashboard dashboard.launch
rosrun pses_control_test pses_control_test
rosrun rqt_reconfigure rqt_reconfigure
```

## Utilities
```
rosbag record /kinect2/qhd/image_color /odom /uc_bridge/hall_cnt /uc_bridge/hall_dt /uc_bridge/hall_dt8 /uc_bridge/imu /uc_bridge/mag /uc_bridge/set_motor_level_msg /uc_bridge/set_steering_level_msg /uc_bridge/usf /uc_bridge/usl /uc_bridge/usr /uc_bridge/vdbat /uc_bridge/vsbat --duration=45 -O file.bag
```

## Authors

* **Kai Cui**
* **Feiyu Chang**
* **Regis Fayard**
* **Lars Semmler**
* **David Botschek**
