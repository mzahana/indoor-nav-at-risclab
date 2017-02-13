# PX4 Software in the loop with Gazebo + ROSR



**NOTE: **This only runs on Linux

[Reference page](https://dev.px4.io/simulation-ros-interface.html)

## Installation

\***MAKE SURE YOU INSTALLED ROS**

[**\*MAKE SURE YOU INSTALLED MAVROS**](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

Remove old gazebo

```bash
sudo apt-get remove gazebo*
```

Add sources for new Gazebo:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update
```

Install **Gazebo7:**

```bash
sudo apt-get install ros-$ROS_DISTRO-gazebo7-ros-pkgs
```

If you have not cloned the PX4 Firmware, do so by following the below commands,

```bash
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

build,

```bash
make posix_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    // (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```



