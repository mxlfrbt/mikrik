![Only_ROS_Noetic](https://img.shields.io/badge/ROS-Noetic-informational)

# mikrik

An open-source two-wheel drive robot that runs Robotics SDK using Realsense camera Mikrik.
Project has two parts:
- Robot part - it runs ROS1 using Raspberry PI 4B. It creates an interface to communicate with the robot motors and encoders data. By finishing that part you will have a real ROS1 robot that can be controlled using a PS4 gamepad. After that you will need to connect it to the host part of the project.
- Host part - it is main brain of the robot. It will generate control commands, and move robot according to the program. It can be based on Intel NUC or Nvidia Jetson. On them you may run ROS2 or ROS1 application that will subscribe to the /cmd_vel topic and will publish value to your robot using ROS1-ROS2 bridge.
- In this tutorial I will briefly cover how to connect robot to the Robotics SDK and run Visual SLAM using Realsense D435(i) camera. Same way you can connect Lidar, but I strongly recommend you to use 3D-cameras or even better an RGB cameras. RGB camera you can teach using AI to detect depth of the scene, but AI part is not covered in the tutorial, if you're a smart guy, you're welcome to contribute.

# Please support the development and buy mikrik robot

If you feel that building a robot is going to be a challenge, please write me an email. You can order a chassis according to your needs. It can be a Ready-To-Run robot with all electronics included, or a chassis kit to build by yourself.


# Robot Assembly

- Detailed instructions for assembling the robot are published on hackster.io. Looking for a volunteer to help.
- My robot is based on CAD-models attached in my mikrik-robot-cad repo. You can use laser-cutter to make basic parts,
and then glue them together.


# Robot packages

The basis of the robot:

- `mikrik_description` — URDF robot description and main bringup launch files.
- `mikrik_driver` — Low-level drivers for working with motors and encoders on Raspberry Pi.
- `mikrik_control` — Robot controllers, differential drive controller.
- `mikrik_base` — Robot base controller.
- `mikrik_teleop` — Remote control of the base robot controller via the DualShock 4 joystick.

# Software requirements

Robot running on Ubuntu 20.04 Server arm64 and ROS Noetic and Raspberry Pi 4B 4 Gb.

## Basis part requirements

Necessary software to run the main part of the project:

- [WiringPi](https://github.com/WiringPi/WiringPi)
- [ds4drv](https://github.com/naoki-mizuno/ds4drv)

ROS packages:

```bash
ros-noetic-gmapping
ros-noetic-joy
ros-noetic-pid
```