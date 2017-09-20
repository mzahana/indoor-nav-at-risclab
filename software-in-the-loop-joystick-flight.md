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
  # or, if you get errors like, cannot create directories...., try,
  # sudo ./install_geographiclib_datasets.sh
  # then, go back to main folder
  cd ~/catkin_ws
  ```

  Then, rebuild MAVROS, `catkin build`

* [Nuttx based Hardware](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#nuttx-based-hardware). Go through all installation instructions. If you get some messages about not found directorires, ignore it.

* Install QGroundcotrol from [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux). Use the AppImage option.

* Now, you need to clone the autopilot firmware source, PX4.
```sh
cd ~
mkdir src
git clone https://github.com/PX4/Firmware.git
```

## Testing SITL with Gazebo (No ROS)
In this step, we will validate that the PX4 SITL app and gazebo work as expected. To run the SITL app and Gazebo, execute the following commands in a new terminal
```
cd ~/src/Frimware
make posix_sitl_default gazebo
```
After sometime, you should be able to see an Iris model loaded in gazebo, and the ```pxh> ``` command line in the terminal. Just hit <Enter couple of times if you don't see the ```pxh> ``` command line, and it should appear.
To takeoff/land the quadcopter, execute the following commands in the terminal

```
pxh> commander takeoff
pxh> commander land
```

If the previous actions succeed the the installation is OK. Next, we will run ROS and a MAVROS node which will allow us to interface the autopilot with ROS.

## Iterfacing with ROS
Assuming that you already created your ```catkin_ws```, we will create symbolic links to the PX4 autopilot and the PX4 simulation package folders into our ROS workspace. This makes it easy to launch everything (Gazebo+PX4 app+ ROS+MAVROS) at once.
```
cd ~/catkin_ws/src
ln -s ~/src/Firmware/ px4
ln -s ~/src/Firmware/Tools/sitl_gazebo/ mavlink_sitl_gazebo
```
## Joystick Package Installation & Usage

This package is needed to interface a joystick to ROS. To install this package, simply execute the following command in the terminal.

```
sudo apt-get install ros-kinetic-joy
```

You will need to setup permissions before you can use your joystick. For that, please find details here.

