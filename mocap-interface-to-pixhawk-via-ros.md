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
  roslaunch
  ```



