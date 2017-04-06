# MOCAP Interface to Pixhawk via ROS

This tutorial explains how to get OptiTrack data to ROS, and feeding this data to Pixhawk for indoor positioning.

![](/assets/Screen Shot 2017-04-06 at 11.07.45 AM.png)

## Prerequisites

* Wifi router \(recommended 5Ghz\)
* MOCAP machine connected to router \(wired or wirless\)
* Linux machine with ROS \(connected to router\)

* ODROID that its connected to Pixhawk via serial interface, with mavros installed as described below. It also should be connected to the WiFi router via a WiFi dongle \(recommended 5Ghz\)

* Pixhawk with PX4 firmware, using LPE estimator. **NOTE**: EKF2 estimator does not support MOCAP yet.

## Setup

* On the Linux machine, clone the `optitrack` ROS package

  ```sh
  pip install optirx --user
  cd catkin_ws/src
  git clone https://github.com/risckaust/optitrack.git
  git clone https://github.com/gt-ros-pkg/hrl-kdl.git -b indigo-devel
  cd optitrack
  git checkout risc_branch
  cd ..

  rosdep update
  rosdep install --from-paths . --ignore-src --rosdistro indigo -y
  cd ~/catkin_ws
  catkin build

  source devel/setup.bash
  ```

* Do this step on the Linux machine and ODROID.  
  Install **mavros** and **mavlink** packages from source. If the binaries are installed using `apt-get install`, make sure you uninstall them using `apt-get remove`

  ```bash
  cd ~/catkin_ws
  catkin init
  wstool init src
  # we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
  rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

  # Install MAVROS: get source (upstream - released)
  rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

  # Create workspace & deps
  wstool merge -t src /tmp/mavros.rosinstall
  wstool update -t src -j4
  rosdep install --from-paths src --ignore-src -y

  cd src/mavros
  git checkout mocap_timestamp
  cd ../..

  catkin build

  #Make sure that you use setup.bash or setup.zsh from workspace.
  #    Else rosrun can't find nodes from this workspace.
  source devel/setup.bash
  ```

* On Linux machine, run optitrack node to get mocap data into ROS,

  ```sh
  roslaunch optitrack optitrack_pipeline.launch iface:=eth1
  ```

  adjust `eth1` according to your network interface. You can check using `ifconfig` command.

In this case, the linux machine is ROS master

* on ODROID, run mavros, assuming roscore is running on the linux machine with, for example, `IP=192.168.1.12`

  ```sh
  export ROS_MASTER_URI:=http://192.168.1.12:11311
  roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@192.168.1.12
  ```

* on the linux machine, in separate terminal,  run the intermediate node which transfer mocap date from optitrack node to mavros,

  ```sh
  rosrun optitrack mocap_pose.py
  ```

Now, Pixhawk should be able to get mocap data

