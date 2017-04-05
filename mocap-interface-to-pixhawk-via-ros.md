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

* install mavros and mavlink packages from source. If the binaries are installed using `apt-get install`, make sure you uninstall them using `apt-get remove`

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

  catkin build

  #Make sure that you use setup.bash or setup.zsh from workspace.
  #    Else rosrun can't find nodes from this workspace.
  source devel/setup.bash
  ```



