# Hector Simulation Software in ROS+IsaacSim

## HECTOR: Humanoid for Enhanced ConTrol and Open-source Research

This branch is under development of the ROS+Isaac simulation for the Hector humanoid robot. \
For humanoid ROS model with arms use the ROS_Humanoid_Simulation branch.

<img src="https://github.com/DRCL-USC/Hector_Simulation/blob/Matlab_Simulation/STL%20files/Hector_picture.jpg?raw=truev" width=50% height=50%>

Introduction Video: https://youtu.be/NcW-NFwjMh0

Video associated with preprint "Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research": https://youtu.be/-r0QoxQgshk

## Dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
* [ROS](http://wiki.ros.org/) Neotic
* [Gazebo](https://gazebosim.org/home) 11
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) (>3.3)
* unitree_legged_sdk 
* [qpOASES](https://github.com/coin-or/qpOASES)
* ROS_Packages

Install dependencies via apt
```
sudo apt-get install -y liblcm-dev ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

## System Requirements:
* x86 platform with nvidia GPU
* Ubuntu20.04 with ROS1 noetic installed

## Build:
```bash
cd ~/catkin_ws
catkin build
```

### launch and run simulation:
1. Run Isaac

2. Run ros controller manager
```bash
roslaunch unitree_gazebo biped_isaac.launch
```
The robot should be standing on the ground

3. Run hector_controller
```bash
roslaunch hector_control hector_control.launch
```

* Click the start button at the bottom of the simulator, the robot should stand up/move away
* In some occasions the controller does not kick in after starting, please terminate the controller with ctrl + \\. Then go back to the simulator, pause, and reset (ctrl + R). Rerun controller. 

## Keyboard Control: 
* Inside the terminal window, use W or S to control x direction speed
* Use A or D to control robot turning 
* Use J or L to control y direction speed

## Cite Us:
Thank you for choosing our software for your research and development, we highly appreciate your citing our work:

1. HECTOR Project: "Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research"
https://arxiv.org/pdf/2312.11868.pdf
```
@article{li2023dynamic,
  title={Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research},
  author={Li, Junheng and Ma, Junchao and Kolt, Omar and Shah, Manas and Nguyen, Quan},
  journal={arXiv preprint arXiv:2312.11868},
  year={2023}
}
```

2. Force-and-moment-based Locomotion MPC: "Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots"  https://arxiv.org/abs/2104.00065
``` 
  @inproceedings{li2021force,
  title={Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots},
  author={Li, Junheng and Nguyen, Quan},
  booktitle={2021 60th IEEE Conference on Decision and Control (CDC)},
  pages={1024--1030},
  year={2021},
  organization={IEEE}
}
```


## Contact Information:
Junheng Li -- junhengl@usc.edu
Omar Kolt -- kolt@laser-robotics.com

## License 
Please read the License.md for details. 

## Acknowledgement：
The authors would like to express special thanks to MIT Biomimetic Lab for providing the [cheetah MPC framework](https://github.com/mit-biomimetics/Cheetah-Software) and Unitree Robotics for providing the Unitree gazebo simulation framework.

