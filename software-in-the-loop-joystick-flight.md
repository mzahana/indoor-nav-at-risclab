# Software-in-the-Loop Joystick Flight

This tutorial explains the steps required to fly a simulated quadrotor in the Gazebo simulator using a real joystick. The following diagram shows how the system components work together.

![](/assets/sitl_diagram.png)

## Hardware Requirements

* Desktop linux machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 14.04 installed
* Joystick

## Software Requirements

* **Ubuntu 14.04**
* **ROS indigo** \(full desktop installation\)
* **Gazebo 7**: You will need to remove the old Gazebo which comes with ROS Indigo, and install Gazebo 7

* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware

* **MAVROS** package: Autopilot ROS interface
* **Joy** package: Joystick ROS interface



In this tutorial, it is assumed that the reader is familiar with basic linux commands, ROS basics, ROS & Gazebo are installed.

NOTE: Although ROS Indigo is becoming an old version, we will continue to use it as all our tested software were built upon ROS indigo. The tutorial will be updated in case a newer ROS version is validated.

## MAVROS Installation

Pleas follow the Binary Installation instructions to install mavros. Remember to change ROS distribution from Kinetic

