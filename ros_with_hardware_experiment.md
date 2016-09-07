# ROS with Hardware Experiment

This tutorial will demonstrate the following,
* Install MAVROS package
* Connect to Pixhawk by running the ```mavros_node```
* Writing ```offb_node``` to control quadcopter to follow circular trajectory.

## Install MAVROS
* We asume that ROS-Indigo is used.
* Do the installation on both the onboard ODROID, and the ground station Linux machine

Log-in to the ODROID account using the ```ssh``` command:
```sh
# account name here is, odroid
ssh odroid@192.168.1.10
# then enter the user name and password
```
First open a terminal, then type the following commands:
```sh
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```

Now, installation is done!

You can check that MAVROS is running properly, by:

first run ```roscore``` in separate terminal

```sh
roscore
```
then, in a separate terminal, run the main node,
```sh
rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600
```