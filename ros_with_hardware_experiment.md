# ROS with Hardware Experiment

<div class="warning">
NOTE: This tutorial session assumes that you have attended the 'ROS Basics' session, and got familiar with ROS.
</div>

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
* make sure ODROID is conencted to the same network router as your PC
* Log-in to the ODROID account using the ```ssh``` command:
```sh
# account name here is, odroid
# choose the correct IP
ssh odroid@192.168.1.10
# then enter password: odroid
```
* Once logged, type the following commands to install MAVROS:
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
* Make sure that Pixhawk is connected to ODROID
* then, in a separate terminal, run the main node, to connect to Pixhawk
```sh
rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600
```
* If the connection failed, you might need to have permission for the serial port,
```sh
sudo chmod a+rw /dev/ttyACM0
# enter password if asked: odroid
```
The serial port might have different address e.g. ```/dev/ttyS2```. Also, make sure that you are using the right baudrate.

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

```c
/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include "math.h"

double r=1.0; // could be parameter in ROS?
double theta;
double count=0.0;
double wn=1.0;// could be parameter in ROS?

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    ROS_INFO("%f",msg->orientation.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        theta = wn*count*0.05;

        pose.pose.position.x = r*sin(theta);
        pose.pose.position.y = r*cos(theta);
        pose.pose.position.z = 15;
        
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
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

```c++
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```
The ```mavros_msgs``` package contains all of the custom messages required to operate services and topics provided by the mavros package. All services and topics as well as their corresponding message types are documented in the [mavros wiki](http://wiki.ros.org/mavros)
```c++
double r=1.0; // could be parameter in ROS?
double theta;
double count=0.0;
double wn=1.0;// could be parameter in ROS?
```
Here we set the circular trajectory parameters. ```r``` is the circle radius, ```wn``` is the frequency.
```c++
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```
We create a simple callback which will save the current state of the autopilot. This will allow us to check connection, arming and offboard flags
```c++
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```
We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change. Note that for your own system, the "mavros" prefix might be different as it will depend on the name given to the node in it's launch file.
```c++
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```
The px4 flight stack has a timeout of 500ms between two offboard commands. If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering offboard mode. This is why the publishing rate must be faster than 2 Hz to also account for possible latencies. This is also the same reason why it is recommended to enter offboard mode from POSCTL mode, this way if the vehicle drops out of offboard mode it will stop in its tracks and hover.
```c++
// wait for FCU connection
while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```
Before publishing anything, we wait for the connection to be established between mavros and the autopilot. This loop should exit as soon as a heartbeat message is received.
```c++
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```
Even though the px4 flight stack operates in the aerospace NED coordinate frame, mavros translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
```c++
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```
Before entering offboard mode, you must have already started streaming setpoints otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
```c++
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```
We set the custom mode to ```OFFBOARD```. A list of [supported modes](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) is available for reference.
```c++
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.success){
                        ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
        } else {
                if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                                arm_cmd.response.success){
                                ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                }
        }

        theta = wn*count*0.05;

        pose.pose.position.x = r*sin(theta);
        pose.pose.position.y = r*cos(theta);
        pose.pose.position.z = 15;
        
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
}
```
The rest of the code is pretty self explanatory. We attempt to switch to offboard mode after which we arm the quad to allow it to fly. We space out the service calls by 5 seconds so as to not flood the autopilot with the requests. In the same loop we continue sending the requested pose at the appropriate rate.

Also, we calculate a new point on the circular trajectory using,
```c++
        theta = wn*count*0.05;

        pose.pose.position.x = r*sin(theta);
        pose.pose.position.y = r*cos(theta);
        pose.pose.position.z = 15;
```

