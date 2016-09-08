# ROS with Hardware Experiment

This tutorial will demonstrate the following,
* Install MAVROS package
* Connect to Pixhawk by running the ```mavros_node```
* Writing ```offb_node``` to control quadrotor (equipped with Pixhawk) to follow circular trajectory.

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
* In the ros workspace you created in the basics tutrorial [**add link**], create a new package:
```sh
# navigate to the src foler
cd ~/ros_ws/src
# create a package. e.g. px4_offb_test
catkin_create_pkg px4_offb_test std_msgs rospy roscpp mavros mavros_msgs
```
* create a ```offb_node.cpp``` file in the package src folder
```sh
cd ~/ros_ws/src/px4_offb_test/src
gedit offb_node.cpp
```
* add the following code to the file,
```sh
# the code goes here!
```
* add the node to the ```CMakeLists.txt```. First, open the CMakeLists file
```sh
cd ~/ros_ws/src/px4_offb_test
gedit CMakeLists.txt
```
* add the following line to the end of this file:
```sh
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
```
* build the ros workspace:
```sh
cd ~/ros_ws
catkin_make
```
## Code explained