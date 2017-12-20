This is a tutorial for RISC lab boot camp. Please find all the components in RISC shoebox and toolbox prepared in Area C’s shelf.

## Basic principles {#Basic-principles}

* Feel free to place the components anywhere but take care of wirings. Refer to quads we already have in the lab.
* Carefully choose zipties, shrinking tubes, duct tapes or soldering for different situations. Generally, for fixing motor wires we use 
  **zipties**
  . Shrinking tubes are for permanent connection between wires while soldering then together.
* Pay attention to 
  **calibration**
  , and prepare for 50% possibility to fail:-\)

## Preliminaries {#Preliminaries}

This tutorial assumes you have the following skills:

* ROS

If not, please refer to a  [ o nline course](http://www.rsl.ethz.ch/education-students/lectures/ros.html) produced by ETHZ. And refer to [solutions to the exercises](https://github.com/luym11/ros_practise) on my github. 

* Soldering

If not, please refer to [basic skill video](https://www.youtube.com/watch?v=BLfXXRfRIzY).

* Basic knowledge about LiPo batteries \(Answer the following questions\)

  * What do 3s, 4s mean?
  * What does 20c mean?
  * What does 1400mAh mean?
  * What are the parameters of your battery?
  * How to charge LiPo battery? How to measure it voltage using meter?
  * What’s the minimum voltage to use a LiPo on the quad? \(IMPORTANT\)

* Basic knowledge about motors \(Answer the following questions\)

  * Different types of motors. \(We are using brushless motor for quads\)
  * What does KV2200 means? What will be changed if KV number grows?
  * What are the parameters of your motors?

## Hardware assembly {#Hardware-assembly}

### Introduction {#Introduction}

Take a look at your RISC shoebox.

* Frame
* Power distribution board

Distribute power from battery to 4 ESCs.

* Pixhawk

Autopilot. It contains IMU and flight controller.

* Power wires

Connect batteries. Distribute power to Pixhawk and other components such as Odroid.

* Motors and propellers

While installing take a look at their directions\(CW vs CCW\).

* Electronic speed controller\(ESC\)

Power and control brushless motors using PWM. It takes DC from power distribution board and convert it to controlled three phase AC, and also it takes control signals from Pixhawk via signal wires \(look like jumper wires\).

* RC receiver

Used with RC to control the quad using RC.

* BEC

Convert voltage to power Odroid.

### Assembly process {#Assembly-process}

* Assemble the frame. Attach the power distribution board to it.
* Mount the motors. Mind CW and CCW. They should be mounted as follows. DON’T install the propellers now.
 
  ![](https://luym11.github.io/images/quad_1.jpg "Motor")

Note that the “front” direction of the quad needs to be decided. Take a look at the Pixhawk and there’s an arrow and it should be pointed at “front” when mounted. And it should also be the direction between 1 and 3 in above picture. We are using “X” configuration.

* Connect ESCs to motors and plug ESCs to power distribution board.
* Install power wire. Find a place to fix it. One end of it should be plugged to power distribution board and the other end is for battery \(DON’T plug it now\).
* Install Pixhawk above the power distribution board but seperate from it.
* There is a wire for powering Pixhawk in the powerwires. Plug it to the power port of the Pixhawk.
* Install buzzer and switch to Pixhawk using their own ports.
* Plug control wires of ESCs to Pixhawk.
  * Mind the direction of jumper wires
  * The numbers on those pins on Pixhawk correspond to the motor numbers of motors in the picture.
* Bind the RC receiver with an RC \(Use Spectrum RC because those receivers in the shoebox should be used with them\) and install them. We have one special binder to do that and it’s placed in the toolbox beside RISC shoeboxes. You will also find RCs there. The binding process is as follows:
  * Insert the jumper wire to Pixhawk like a motor, we just need to power it through this way
  * Insert the signal wire to another end of the RC receiver
  * Plug the battery \(Find in the toolbox\) to power wire
  * Turn on the RC while pressing the left up button \(marked as Trainer Bind\), wait until the RC receiver is lighting statically
  * Remove the RC from binder, use its own signal wire to connect with Pixhawk via port SPKT/DSM
* Plug the battery and check 4 ESCs has static green LED lighted up and BEC has red. Buzzer will produce sound in the beginning and remain slient. Unplug the battery.
* For this stage there’s no need to install Odroid. This will be illustrated later.

### RC flying {#RC-flying}

* Download QGroundControl on your PC and open it.
* Connect Pixhawk with your PC via USB. You should see it connected in the software.
* Airframe tab
  * Choose proper airframe according to the one you have.
* Radio tab
  * Click “Calibrate” button and follow instructions.
* Sensors tab
  * Calibrate all the sensors following the instructions. After calibration they will be green.
  * Note that there will be a progress bar while calibrationg each sensor, which indicates your progress. After finishing calibration of one sensor, wait until it reaches the end and then press “OK” and move on to the next.
* Flight Modes tab
  * Modes: Channel 6 \(marked as FLAP/GYRO\)
    * Mode 1: Position
    * Mode 4: Altitude
    * Mode 6: Manual
  * Kill switch: Channel 5 \(GEAR MIX\)
  * If you set them right, when you are picking those channels your action will be reported in QGroundControl
* Power tab
  * Write the parameters of your battery \(Number of cells, Full/ Empty voltages\)
  * Click “Calculate” beside “Voltage divider” and type in measured voltage \(can be measured using voltage meter provided in the toolbox\), click “Calculate”
  * Click “Calibrate” to calibrate ESCs
* Parameters tab

  * General process is tuning P first then I and D at last for yaw, pitch and roll. Hold the quad in your hand to feel if it’s well tuned. 
    [Guide](https://docs.px4.io/en/advanced_config/pid_tuning_guide_multicopter.html)
  * You can plug other quads we have to your PC and export the parameters there and load them to your quad directly

* Flying

  * Unplug the quad from PC
  * Press Safety switch on Pixhawk until it blinks faster
  * Arm \(Hold throttle at minimum and rudder to the right for 2 seconds\) the quad using RC and check if the motors are spinning in correct directions as the picture by slightly touching them. If not, change any two wires of that motor. Disarm \(Opposite of arm\)
  * Install propellers \(Note there are CW and CCW propellers as well\)
  * \(optional\) If still want to connect the quad to QGroundControl, install telemetry
  * Manual kill switch off. Select manual mode.
  * Check battery condition. \(Always keep this in mind\)
  * Put the quad in the cage and arm again. Slowly add throttle while keep it in the middle of the cage by controlling pitch and yaw.
  * If it flies, you can do more manoeuvres

## Trouble shooting {#Trouble-shooting}

### Motors not rotating while armed and rotates with higher throttle {#Motors-not-rotating-while-armed-and-rotates-with-higher-throttle}

Check PWM min/max in parameters and make sure it’s associated with ESCs


**Wrriten by Yimeng Lu**
