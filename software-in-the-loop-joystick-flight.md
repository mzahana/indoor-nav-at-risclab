# Software-in-the-Loop Joystick Flight

This tutorial explains the steps required to fly a simulated quadrotor in the Gazebo simulator using a real joystick. The following diagram shows how the system components work together.

![](/assets/sitl_diagram.png)

## Hardware Requirements

* Desktop linux machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 14.04 installed
* Joystick

## Software Requirements

* **Ubuntu 16.04**
* **ROS Kinetic** \(full desktop installation\)
* **Gazebo 7**: will be automatically installed with ROS

* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware

* **MAVROS** package: Autopilot ROS interface

* **Joy** package: Joystick ROS interface

**NOTE: In this tutorial, it is assumed that the reader is familiar with basic linux commands, ROS basics, ROS & Gazebo are installed.**



## Setup Steps

Please follow the following steps carefully to setup the development enviroment which includes the software-in-the-loop simulation. It is assumed that the following steps are done on a clean Ubuntu 16.04 OS image.



Open a new terminal window \(you can hit ctrl+alt+t to bring up one\). then, please follow the steps in this [link](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#permission-setup), starting by the Persmission Setup section. Cover the installation instructions in following sections.

* [Permission setup](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#permission-setup)
* [Remove the modem manager](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#remove-the-modemmanager)
* [Ninja Build System](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#ninja-build-system)
* [Common Dependencies](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies)
* [FastRTPS instalation](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#fastrtps-installation)
* [jMAVSim](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#jmavsim)
* Skip Gazebo section as it will be done in next step
* [ROS/Gazebo](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#rosgazebo)
* MAVROS installation from source. After you build MAVROS using `catkin build` , install the GeographicLib
  ```
  cd ~/catkin_ws/src/mavros/mavros/scripts
  ./install_geographiclib_datasets.sh
  # or, if you get errors like, cannot create directories....
  # sudo ./install_geographiclib_datasets.sh
  # go back to main folder
  cd ~/catkin_ws
  ```

  Then, rebuild MAVROS, ` catkin build`

* 


## MAVROS Installation

Pleas follow the [Binary Installation](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb) instructions to install mavros. Remember to change ROS distribution from Kinetic to Indigo

```
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```

## Joystick Package Installation & Usage

This package is needed to interface a joystick to ROS. To install this package, simply execute the following command in the terminal.

```
sudo apt-get install ros-indigo-joy
```



