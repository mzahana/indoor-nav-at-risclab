# PX4 Software in the loop with Gazebo + ROS

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



