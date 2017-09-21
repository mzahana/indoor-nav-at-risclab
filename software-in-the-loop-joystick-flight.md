# Software-in-the-Loop Joystick Flight

This tutorial explains the steps required to fly a simulated quadrotor in the Gazebo simulator using a real joystick. The following diagram shows how the system components work together.

![](/assets/sitl_diagram.png)

## Hardware Requirements

* Desktop linux machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 16.04 installed
* Joystick

## Software Requirements

* **Ubuntu 16.04**
* **ROS Kinetic** \(full desktop installation\)
* **Gazebo 7**: will be automatically installed with ROS

* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware

* **MAVROS** package: Autopilot ROS interface

* **Joy** package: Joystick ROS interface

**NOTE: In this tutorial, it is assumed that the reader is familiar with basic linux commands, ROS basics.**

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
cd src
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

## Interfacing with ROS
Assuming that you already created your ```catkin_ws```, we will create symbolic links to the PX4 autopilot and the PX4 simulation package folders into our ROS workspace. This makes it easy to launch everything (Gazebo+PX4 app+ ROS+MAVROS) at once.
```
# go to the workspace folder
cd ~/catkin_ws/src
# create symlink to the px4 package
ln -s ~/src/Firmware/ px4
# create symlink to the simulatio package
ln -s ~/src/Firmware/Tools/sitl_gazebo/ mavlink_sitl_gazebo
# re-build your workspace
cd ~/catkin_ws
catkin build
# always source your workspace after each build, so changes take effect.
source devel/setup.bash
```

Now, you are ready to launch Gazebo+PX4 SITL app+ROS+MAVROS. To do that, execute the following command.
```
roslaunch px4 mavros_posix_sitl.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
**TO BE DONE**: explain the previous command.

You should be able to see ```/mavros``` topics using ```rostopic list``` in a new terminal. Also if you execute ``` rosnode list``` in a new terminal, yu should see
```
$ rosnode list
/gazebo
/mavros
/rosout
```
To double check that MAVROS node is connected properly to the PX4 SITL app, try to ```echo``` some topics *e.g.*
```
rostopic echo /mavros/imu/data
```
you should see the ```imu``` data changing.

Now, you can monitor the dorne's states and control it via a mavros node.
* In this tutorial, we are going to control the quadcopter's position via a joystick.
* There is a flight mode in PX4 autopilot which is called **OFFBOARD** mode. This mode allows the autopilot to accept specific external commands such as position, velocity, and attitude setpoints. You cannot mix between different setpoints *e.g.* velocity setpoints in x/y and position in z.
* A MAVROS node provides setpoint plugins which will listen to a user input on specific setpoint topics. Once the user publishes to those specific setpoint topics, the mavros node will transfer thos setpoints to the autopilot to execute.
* If the autopilot's flight mode is **OFFBOARD**, the autopilot will accept the received setpoints and execute them.
* The setpoint topic that we will use in this tutorial is 
```
/mavros/setpoint_raw/local
```
This topic accepts both position and velocity setpoints according to a specific flag. Next, we will create our custom simple ROS package in which we create a simple ROS node that listens to joystic commands from a ROS topic. Then, it will convert joystic commands to position setpoints which will be published to the ```/mavros/setpoint_raw/local`` topic.

We need to install one more package. Go ahead to the following section

## Joystick Package Installation & Usage

This package is needed to interface a joystick to ROS. To install this package, simply execute the following command in the terminal.

```
sudo apt-get install ros-kinetic-joy
```

You will need to setup permissions before you can use your joystick.
* Plug a joystick
* Check if Linux recognizes your joystick
```
ls /dev/input/
```
You will get an output similar to the follwing.
```
by-id    event0  event2  event4  event6  event8  mouse0  mouse2  uinput
by-path  event1  event3  event5  event7  js0     mice    mouse1
```
As you can see, the joystick device is referred to as ```jsX``` where ```X``` is the number of the joystick device.
* Let's make the joystick accessible to the joy ROS node.
```
 ls -l /dev/input/jsX
```
You will see something similar to:

    ```
    crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX
    ```
If XX is rw: the js device is configured properly.
If XX is --: the js device is not configured properly and you need to: 
```
sudo chmod a+rw /dev/input/jsX
```
* test the ```joy``` node. First, start ```roscore``` in a terminal. In another terminal,
```
# set the device address
rosparam set joy_node/dev "/dev/input/js0"
# run the joy node
rosrun joy joy_node
```
In another terminal, echo the ```joy``` topic and move the joystick to see the topic changes
```
rostopic echo /joy
```
You should see an output similat to the following.
```
header: 
  seq: 699
  stamp: 
    secs: 1505985329
    nsecs: 399636113
  frame_id: ''
axes: [-0.0, -0.0, -0.8263657689094543]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

Now, let's write a custom node that reads joystick's commands and convert them to position setpoints to control the quadcopter's poisiton in Gazebo.

## Custom Setpoint Node
Now, it's time for some coding!
You will write a ROS node in Python that listens to the ```/joy``` topic that is published by the ```joy``` node, and convrets the joystick commands to xyz position setpoints. Then, it will publish the calculated position setpoints into ```/mavros/setpoint_raw/local```

Publishing to ```/mavros/setpoint_raw/local``` topic is not enough to get the autopilot to track the setpoints. It has to be in **OFFBOARD** mode. So, in your custom node, you will have to send a signal to activate this mode, only once. You need to **remember** that for this mode to work, you will need to be publishing setpoints beforehand, then, activate it, and continue publsihing setpoints. **If you don't publish setpoints at more than 2Hz, it will go into a failsafe mode**.