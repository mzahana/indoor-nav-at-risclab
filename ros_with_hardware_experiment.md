# ROS with Hardware Experiment

This tutorial will demonstrate the following,
* Install MAVROS package
* Connect to Pixhawk by running the ```mavros_node```
* Writing ```offb_node``` to control quadrotor to follow circular trajectory.

## Install MAVROS
<div class="info">
NOTE: We asume that ROS-Indigo is used.
</div>
* We will install MAVROS on ODROID
* Make sure that ODROID has internet conncetion
* make sure ODROID is conencted to same network router as your PC
* Log-in to the ODROID account using the ```ssh``` command:
```sh
# account name here is, odroid
ssh odroid@192.168.1.10
# then enter password: odroid
```
* Once logged, type the following commands to install ROS-indigo:
```sh
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
# enter your account password if asked
```
Now, installation is done!

## Connect to Pixhawk
<div class="info">
NOTE: We assume that ODROID is connected serially to Pixhawk via TELEM2 port
</div>
**Note**: The following steps are done on ODROID.

* Log into ODROID using ```ssh``` as explained above.
* You can check that MAVROS is running properly, by:
first run ```roscore``` in separate terminal
```sh
roscore
```
* then, in a separate terminal, run the main node, to connect to Pixhawk
```sh
rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600
```

## Writing off-board node
**Note**: The following steps are done on ODROID.