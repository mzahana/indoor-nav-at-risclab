# Setup HIL with PX4 and V-REP

 ## Prerequisites
 * Machine with Ubuntu 14.04 LTS installed
 * [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) installed
 * CATKIN workspace
 * [V-REP](http://www.coppeliarobotics.com/downloads.html) installed
 * V-REP HIL scene
 * Pixhawk loaded with PX4 HIL firmware
 * setup script ```vrep_px4_hil_setup.sh```
 * Internet connection

## Setup
Open a new terminal, define the setup variables: ```VREP_ROOT``` is the VREP main folder's path, ```ROS_WORKSPACE``` is the path to your catkin workspace. Finally, run the setup script. Make sure you have internet connection.

```sh
# export VREP_ROOT=path/to/vrep/folder
# export ROS_WORKSPACE=path/to/catkin/workspace
./vrep_px4_hil_setup.sh
```

Once the installation is successful, connect Pixhawk via USB. Run ```mavros``` and connect to Pixhawk,
```sh
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@192.168.1.135
```
In another terminal, run V-REP. Navigate to VREP main folder, then execute
```sh
./vrep.sh
```

Load the ```px4_hil.ttt``` scene, and run it. You show see the main LED on Pixhawk go green. It means it's able to get xyz data (fake GPS).