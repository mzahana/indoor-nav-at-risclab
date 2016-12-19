# Setup HIL with PX4 and V-REP

## Prerequisites

* Machine with Ubuntu 14.04 LTS installed
* [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) installed
* CATKIN workspace
* [V-REP](http://www.coppeliarobotics.com/downloads.html) installed
* V-REP HIL scene: can be found in the `catkin_ws/src/v_repExtRosInterface/vrep_hil`  folder after you run the setup .sh script
* Pixhawk loaded with PX4 HIL firmware \(use[ v1.4.4](https://github.com/PX4/Firmware/releases/tag/v1.4.4)\)

* customized .params file for appropriate Pixhawk parameters configurations: can be found in the `catkin_ws/src/v_repExtRosInterface/vrep_hil`  folder after you run the setup .sh script

* setup script `vrep_px4_hil_setup.sh` \(see the code below\)

* Internet connection

## Setup

Prepare the setup file as described in the below section.

Open a new terminal, navigate to the setup file, and define the setup variables: `VREP_ROOT` is the VREP main folder's path, `ROS_WORKSPACE` is the path to your catkin workspace. Finally, run the setup script \(see the code below\). **Make sure you have internet connection and root access via **`sudo`.

```sh
# export VREP_ROOT=path/to/vrep/folder
# export ROS_WORKSPACE=path/to/catkin/workspace
./vrep_px4_hil_setup.sh
```

Once the installation is successful, connect Pixhawk via USB. Run `mavros` to connect to Pixhawk,

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@192.168.1.135
```

You may need to adjust `fcu_url` address `/dev/ttyACM0:115200` and `gcs_url` address `udp://@192.168.1.135` according to your setup.

In another terminal, run V-REP:  Navigate to VREP main folder, then execute

```sh
# cd /vrep/folder
./vrep.sh
```

Load the `px4_hil.ttt` scene, and run it. You should see the main LED on Pixhawk go green. It means it's able to get xyz data \(fake GPS\).

## Setup Shell Script

You can create the setup file by copying the following shell code to a file, and then, run it. Make sure it has `.sh` extension, and make it executable : `chmod +x <filename.sh>`.

```bash
#!/bin/bash

# Check if required environment variables are set properly
if [ ! -v ROS_WORKSPACE ]; then
    echo "!!!! ERROR: ROS_WORKSPACE is unset"
    echo "set it using: export ROS_WORKSPACE=path/to/workspace/folder"
    echo "press 'ENTER' to exit....."
    read x
    exit 1
fi

if [ ! -v VREP_ROOT ]; then
    echo "!!!! ERROR: VREP_ROOT is unset"
    echo "set it using: export VREP_ROOT=path/to/VREP/folder"
    echo "press 'ENTER' to exit....."
    read x
    exit 1
fi

ROS_WORKSPACE1=$(echo $ROS_WORKSPACE | tr -d '\r')
VREP_ROOT1=$(echo $VREP_ROOT | tr -d '\r')

# Clean ros_ws: build/devel/logs directories
cd $ROS_WORKSPACE1
rm -r -f devel/
rm -r -f build/
rm -r -f logs/

# Initialize catkin workspace
cd "$ROS_WORKSPACE1"
catkin init
cd src
rm .rosinstall
cd ..
wstool init src

# Remove old vrep ros interface package`
cd "$ROS_WORKSPACE1/src"
if [ -d "v_repExtRosInterface" ]; then
    rm -r -f "$ROS_WORKSPACE1/src/v_repExtRosInterface"
fi

# Remove old mavros package
# remove mavros if installed by apt-get
sudo apt-get remove ros-indigo-mavros
sudo apt-get remove ros-indigo-mavros-extras
sudo apt-get remove ros-indigo-mavros-msgs
if [ -d "mavros" ]; then
    rm -r -f mavros
fi

# Remove mavlink package
# remove mavlink if installed by apt-get
sudo apt-get remove ros-indigo-mavlink
if [ -d "mavlink" ]; then
    rm -r -f mavlink
fi

# Create Python-packages folder,
cd ~
# check if directory exists
if [ ! -d "python-packages" ]; then
    mkdir -p "python-packages/src"
fi


# Get some required python packages
sudo apt-get update
sudo apt-get install python-tempita python-catkin-tools python-rosinstall-generator python-pip -y
sudo pip install future


# Clone fresh vrep ros interface package
cd "${ROS_WORKSPACE1}/src"
git clone https://github.com/mzahana/v_repExtRosInterface.git
# Copy some V-REP packages from V-REP folder
cp -R "${VREP_ROOT1}/programming/ros_packages/vrep_common/" "${ROS_WORKSPACE1}/src/"
cp -R "${VREP_ROOT1}/programming/ros_packages/vrep_joy/" "${ROS_WORKSPACE1}/src/"
cp -R "${VREP_ROOT1}/programming/ros_packages/vrep_plugin/" "${ROS_WORKSPACE1}/src/"
cp -R "${VREP_ROOT1}/programming/ros_packages/vrep_plugin_skeleton/" "${ROS_WORKSPACE1}/src/"
cp -R "${VREP_ROOT1}/programming/ros_packages/vrep_skeleton_msg_and_srv/" "${ROS_WORKSPACE1}/src/"


# Get fresh mavros package
git clone https://github.com/mzahana/mavros.git
# checkout the px4_hil_plugins branch
cd mavros
git checkout  px4_hil_plugins
cd "${ROS_WORKSPACE1}"

# Get fresh mavlink package
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y



# Get supporting package for vrep ros interface
cd ~/python-packages
# Remove old package if exists
if [ -d "v_repStubsGen" ]; then
    rm -r -f v_repStubsGen
fi

# Get a fresh copy of the supporting python package
git clone https://github.com/fferri/v_repStubsGen.git
export PYTHONPATH=$PYTHONPATH:~/python-packages


# Build ros/catkin workspace
#VERBOSE=1 catkin build -v -p1 -j1 --no-status
#catkin build -p1 -j1
cd "${ROS_WORKSPACE1}"
catkin build


# clone built libs to V-REP folder
cp -r "${ROS_WORKSPACE1}/devel/lib/libv_repExtRosInterface.so" ${VREP_ROOT1}
cp -r "${ROS_WORKSPACE1}/devel/lib/libv_repExtRos.so" ${VREP_ROOT1}
cp -r "${ROS_WORKSPACE1}/devel/lib/libv_repExtRosSkeleton.so" ${VREP_ROOT1}
#cp -r $ROS_WORKSPACE/src/ros_bubble_rob/bin/rosBubbleRob ~/V-REP_PRO_EDU_V3_3_2_64_Linux/
#cp -r $ROS_WORKSPACE/src/ros_bubble_rob2/bin/rosBubbleRob2 ~/V-REP_PRO_EDU_V3_3_2_64_Linux/

source "${ROS_WORKSPACE1}/devel/setup.bash"

### DONE ###
```



