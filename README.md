# Crash-Test-Dummies
This is the PSES implementation repository of the group 'Crash-Test-Dummies'. It implements algorithms to follow colored lanes and react to traffic signals.

## Getting Started

Please feel free to browse the project's documentation, where you can find instructions on how to use and configure this package.

[Pses_docs](https://github.com/tud-pses/pses_docs)

For OpenCV 3.4, you can use the script installOpenCV3_4.sh. 

After installing required dependencies of the submodules, you can start the control node and begin driving. To give the control nodes commands, you can use dynamic reconfiguration, i.e. rqt_reconfigure. For example: 

```
roslaunch pses_ucbridge uc_bridge.launch 
rosrun kinect2_bridge kinect2_bridge
rosrun pses_control_mpc pses_control_mpc
rosrun pses_sign_recognition obj_detection _gui:=true
rosrun rqt_reconfigure rqt_reconfigure
```

You can then use dynamic reconfiguration to set parameters such as the target velocity to begin driving.

## Authors

* **Kai Cui**
* **Feiyu Chang**
* **Regis Fayard**
* **Lars Semmler**
* **David Botschek**
