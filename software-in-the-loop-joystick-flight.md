# Software-in-the-Loop Joystick Flight

This tutorial explains the steps required to fly a simulated quadrotor in the Gazebo simulator using a real joystick. The following diagram shows how the system components work together.

![](/assets/sitl_diagram.png)

## Hardware Requirements

* Desktop linux machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 16.04 installed
* Joystick

## Software Requirements

* **Ubuntu 16.04**
* **ROS Kinetic** \(full desktop installation\)
* **Gazebo 7**: will be automatically installed with ROS

* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware

* **MAVROS** package: Autopilot ROS interface

* **Joy** package: Joystick ROS interface

**NOTE: In this tutorial, it is assumed that the reader is familiar with basic linux commands, ROS basics.**

## Setup Steps

Please follow the following steps carefully to setup the development enviroment which includes the software-in-the-loop simulation. It is assumed that the following steps are done on a clean Ubuntu 16.04 OS image.

Open a new terminal window \(you can hit ctrl+alt+t to bring up one\). then, please follow the steps in this [link](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#permission-setup), starting by the Persmission Setup section. Cover the installation instructions in following sections.

* [Permission setup](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#permission-setup)
* [Remove the modem manager](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#remove-the-modemmanager)
* [Ninja Build System](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#ninja-build-system)
* [Common Dependencies](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies)
* [FastRTPS instalation](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#fastrtps-installation)
* [jMAVSim](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#jmavsim)
* Skip Gazebo section as it will be done in next step
* [ROS/Gazebo](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#rosgazebo)(Kinetic). Skip this if already done.
* [MAVROS installation](https://github.com/mavlink/mavros/blob/master/mavros/README.md#source-installation) from source. Don't forget to install GeographicLib before you build your ```catkin_ws``` install the GeographicLib

  ```
  cd ~/catkin_ws/src/mavros/mavros/scripts
  ./install_geographiclib_datasets.sh
  # or, if you get errors like, cannot create directories...., try,
  # sudo ./install_geographiclib_datasets.sh
  # then, go back to main folder
  cd ~/catkin_ws
  ```

  Then, build your workspace using `catkin build`

* [Nuttx based Hardware](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#nuttx-based-hardware). Go through all installation instructions. If you get some messages about not found directorires, ignore them!

* Install QGroundcotrol from [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux). Use the AppImage option.

* Now, you need to clone the autopilot firmware source, PX4.
```sh
cd ~
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
```

## Testing SITL with Gazebo (No ROS)
In this step, we will validate that the PX4 SITL app and gazebo work as expected. To run the SITL app and Gazebo, execute the following commands in a new terminal
```
cd ~/src/Frimware
make posix_sitl_default gazebo
```
After sometime, you should be able to see an Iris model loaded in gazebo, and the ```pxh> ``` command line in the terminal. Just hit ENTER couple of times if you don't see the ```pxh> ``` command line, and it should appear.
To takeoff/land the quadcopter, execute the following commands in the terminal

```
pxh> commander takeoff
pxh> commander land
```

If the previous actions succeed the the installation is OK. Next, we will run ROS and a MAVROS node which will allow us to interface the autopilot with ROS.

## Interfacing with ROS
Assuming that you already created your ```catkin_ws```, we will create symbolic links to the PX4 autopilot and the PX4 simulation package folders into our ROS workspace. This makes it easy to launch everything (Gazebo+PX4 app+ ROS+MAVROS) from one place at once.
```
# Go to the workspace src folder
cd ~/catkin_ws/src
# create symlink to the px4 package
ln -s ~/src/Firmware/ px4
# Create symlink to the simulation package
ln -s ~/src/Firmware/Tools/sitl_gazebo/ mavlink_sitl_gazebo
# Build your workspace
cd ~/catkin_ws
catkin build
# Always source your workspace after each build, so changes take effect.
source devel/setup.bash
```

Now, you are ready to launch Gazebo+PX4 SITL app+ROS+MAVROS. To do that, execute the following command.
```
roslaunch px4 mavros_posix_sitl.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
**TO BE DONE**: explain the previous command.

You should be able to see ```/mavros``` topics using ```rostopic list``` in a new terminal. Also if you execute ``` rosnode list``` in a new terminal, you should see
```
$ rosnode list
/gazebo
/mavros
/rosout
```
To double check that MAVROS node is connected properly to the PX4 SITL app, try to ```echo``` some topics *e.g.*
```
rostopic echo /mavros/state
```
Which will show if the mavros node is connected to the PX4 SITL app or not.

Now, you can monitor the dorne's states and control it via a mavros node.
* As mentioned, in this tutorial, we are going to learn one basic way of controlling the quadcopter's position via a joystick.
* There is a flight mode in PX4 autopilot which is called **OFFBOARD** mode. This mode allows the autopilot to accept specific external commands such as position, velocity, and attitude setpoints. You cannot mix between different setpoints *e.g.* velocity setpoints in x/y and position in z.
* A MAVROS node provides setpoint plugins which will listen to a user input on specific setpoint topics. Once the user publishes to those specific setpoint topics, the mavros node will transfer thos setpoints to the autopilot to execute.
* If the autopilot's flight mode is **OFFBOARD**, the autopilot will accept the received setpoints and execute them.
* We will send position setpoints to the autopilot via a setpoint topic that is available in MAVROS. Once set points are received in that topic, the mavros node will send it to the autopilot.
* The setpoint topic that we will use in this tutorial is 
```
/mavros/setpoint_raw/local
```
This topic accepts both position and velocity setpoints according to a specific flag. Next, we will create our custom simple ROS package in which we create a simple ROS node that listens to joystic commands from a ROS topic. Then, it will convert joystic commands to position setpoints which will be published to the ```/mavros/setpoint_raw/local`` topic. Finally, MAVROS will take the position set points and send them to the autopilot.

You might be asking, how are we going to get the joystick commands? The next section explains that.

## Joystick Package Installation & Usage

A package named ```joy``` is going to be used to interface a joystick to ROS. To install that package, simply execute the following command in the terminal.

```
sudo apt-get install ros-kinetic-joy
```

You will need to setup permissions before you can use your joystick.
* Plug a joystick
* Check if Linux recognizes your joystick
```
ls /dev/input/
```
You will get an output similar to the follwing.
```
by-id    event0  event2  event4  event6  event8  mouse0  mouse2  uinput
by-path  event1  event3  event5  event7  js0     mice    mouse1
```
As you can see, the joystick device is referred to as ```jsX``` where ```X``` is the number of the joystick device.
* Let's make the joystick accessible to the joy ROS node.
```
 ls -l /dev/input/jsX
```
You will see something similar to:

    ```
    crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX
    ```
If XX is rw: the js device is configured properly.
If XX is --: the js device is not configured properly and you need to: 
```
sudo chmod a+rw /dev/input/jsX
```
* test the ```joy``` node. First, start ```roscore``` in a terminal. In another terminal,
```
# set the joystick device address
rosparam set joy_node/dev "/dev/input/js0"
# run the joy node
rosrun joy joy_node
```
In another terminal, echo the ```joy``` topic and move the joystick to see the topic changes
```
rostopic echo /joy
```
You should see an output similar to the following.
```
header: 
  seq: 699
  stamp: 
    secs: 1505985329
    nsecs: 399636113
  frame_id: ''
axes: [-0.0, -0.0, -0.8263657689094543]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

Now, let's write a custom node that reads joystick's commands and convert them to position setpoints to control the quadcopter's poisiton in Gazebo.

## Custom Setpoint Node
**Now, it's time for some coding!**
You will write a ROS node in Python that listens to the ```/joy``` topic that is published by the ```joy``` node, and convrets the joystick commands to xyz position setpoints. Then, it will publish the calculated position setpoints into ```/mavros/setpoint_raw/local```

Publishing to ```/mavros/setpoint_raw/local``` topic is not enough to get the autopilot to track the setpoints. It has to be in **OFFBOARD** mode. So, in your custom node, you will have to send a signal to activate this mode, only once. You need to **remember** that for this mode to work, you will need to be publishing setpoints beforehand, then, activate it, and continue publsihing setpoints. **If you don't publish setpoints at more than 2Hz, it will go into a failsafe mode**.
* **First, create your custom ROS package.** The code is commented so you can get an idea of what each part does.
```
cd ~/catkin_ws/src
catkin_create_pkg mypackage std_msgs mavros_msgs roscpp rospy
cd mypackage
# usually python scripts (nodes) are placed in a folder called scripts
mkdir scripts
cd scripts
gedit setpoints_node.py
```
* **copy the following code** to the ```setpoints_node.py``` file

```python
#!/usr/bin/env python

# ROS python API
import rospy
# Joy message structure
from sensor_msgs.msg import Joy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

	def setStabilizedMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Offboard Mode could not be set."%e

	def setAltitudeMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def setPositionMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
	   		print "service set_mode call failed: %s. Autoland Mode could not be set."%e

# Main class: Converts joystick commands to position setpoints
class Controller:
	# initialization method
	def __init__(self):
		# Drone state
		self.state = State()
		# Instantiate a setpoints message
		self.sp 		= PositionTarget()
		# set the flag to use position setpoints and yaw angle
		self.sp.type_mask	= int('010111111000', 2)
		# LOCAL_NED
		self.sp.coordinate_frame= 1

		# We will fly at a fixed altitude for now
		# Altitude setpoint, [meters]
		self.ALT_SP		= 3.0
		# update the setpoint message with the required altitude
		self.sp.position.z	= self.ALT_SP

		# Instantiate a joystick message
		self.joy_msg		= Joy()
		# initialize
		self.joy_msg.axes=[0.0, 0.0, 0.0]

		# Step size for position update
		self.STEP_SIZE = 2.0

		# Fence. We will assume a square fence for now
		self.FENCE_LIMIT = 5.0

		# A Message for the current local position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)

	# Callbacks

	## local position callback
	def posCb(self, msg):
		self.local_pos.x = msg.pose.position.x
		self.local_pos.y = msg.pose.position.y
		self.local_pos.z = msg.pose.position.z

	## joystick callback
	def joyCb(self, msg):
		self.joy_msg = msg

	## Drone State callback
	def stateCb(self, msg):
		self.state = msg

	## Update setpoint message
	def updateSp(self):
		x = -1.0*self.joy_msg.axes[0]
		y = self.joy_msg.axes[1]

		self.sp.position.x = self.local_pos.x + self.STEP_SIZE*x
		self.sp.position.y = self.local_pos.y + self.STEP_SIZE*y

# Main function
def main():

	# initiate node
	rospy.init_node('setpoint_node', anonymous=True)

	# flight mode object
	modes = fcuModes()
	# controller object
	cnt = Controller()

	# ROS loop rate, [Hz]
	rate = rospy.Rate(20.0)

	# Subscribe to drone state
	rospy.Subscriber('mavros/state', State, cnt.stateCb)

	# Subscribe to drone's local position
	rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
	# subscribe to joystick topic
	rospy.Subscriber('joy', Joy, cnt.joyCb)
	
	# Setpoint publisher	
	sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


	# Make sure the drone is armed
	while not cnt.state.armed:
		modes.setArm()
		rate.sleep()
	
	# We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
	k=0
	while k<10:
		sp_pub.publish(cnt.sp)
		rate.sleep()
		k = k+1

	# activate OFFBOARD mode
	modes.setOffboardMode()

	# ROS main loop
	while not rospy.is_shutdown():
		cnt.updateSp()
		sp_pub.publish(cnt.sp)
		rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
```

* **Make a launch folder. We will create a ROS laucnh file to run everything at once.**

```
cd ~/catkin_ws/src/mypackage
mkdir launch
cd launch
gedit joystick_flight.launch
```

* **Then, copy the following lines to the launch file**

```
<launch>

	<arg name="joy_dev" default="/dev/input/js0"/>

	<arg name="fcu_url" default="udp://:14540@127.0.0.1:14557" />


	<include file="$(find px4)/launch/mavros_posix_sitl.launch">
		<arg name="fcu_url" value="$(arg fcu_url)" />
	</include>

	<node pkg="joy" type="joy_node" name="joy_node"  required="true" output="screen">
			<param name="dev" type="string" value="$(arg joy_dev)" />
	</node>

	<node pkg="mypackage" type="setpoints_node.py" name="setpoints_node"  required="true" output="screen">
	</node>

</launch>
```

* **In a fresh terminal, you can run the whole system by executing**
```
roslaunch mypackage joystick_flight.launch
```

* **DONE**. Now, you should see a quadcopter in Gazebo flying at a fixed height and responding to your joystick commands.

**NOTE** always make sure that you have joystick permissions configured properly.
