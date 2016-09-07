# ROS with Hardware Experiment

This tutorial will demonstrate the following,
* Install MAVROS package
* Connect to Pixhawk by running the ```mavros_node```
* Writing ```offb_node``` to control quadrotor to follow circular trajectory.

## Install MAVROS
<div class="info">
NOTE: We asume that ROS-Indigo is used.
</div>
1. Log-in to the ODROID account using the ```ssh``` command:
```sh
# account name here is, odroid
ssh odroid@192.168.1.10
# then enter password: odroid
```

2. Once logged, type the following commands:
```sh
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```
Now, installation is done!

## Connect to Pixhawk
<div class="info">
NOTE: We assume that ODROID is connected serially to Pixhawk via TELEM2 port
</div>
You can check that MAVROS is running properly, by:
first run ```roscore``` in separate terminal
```sh
roscore
```
then, in a separate terminal, run the main node, to connect to Pixhawk
```sh
rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600
```

## Writing offboard node