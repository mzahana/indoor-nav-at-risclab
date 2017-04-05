# MOCAP Interface to Pixhawk via ROS

This tutorial explains how to get OptiTrack data to ROS, and feeding this data to Pixhawk for indoor positioning.

## Prerequisites

* Wifi router \(recommended 5Ghz\)
* MOCAP machine connected to router \(wide or wirless\)
* Linux machine with ROS

* ODROID that its connected to Pixhawk via serial interface, with mavros installed as described below. It also should be connected to the WiFi router

## Setup

* On the Linux machine, clone the `optitrack` ROS package
  ```sh
  cd catkin_ws/src
  git clone https://github.com/risckaust/optitrack.git
  git checkout risc_branch
  ```



