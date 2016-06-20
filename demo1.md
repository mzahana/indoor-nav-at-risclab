# Demo1
In this demo we will learn two things,
* Sending high-level commands  from *MATLAB* to Pixhawk, using *MATMAV*
* Getting live stream of images (into *MATLAB*) from ODROID which is mounted on a quadcopter.
![](demo1.png)

As you can see from the previous figure, there are 4 main components to setup.
* Motion capture system.
* Quadcopter with Pixhawk flight controller.
* ODROID: embedded Linux computer.
* MATLAB enviornment.

## Motion capture setup
Motion capture (or Mocap in short) is used to provide accurate positions and orientations in an indoor environment. The mocap setup we have in the lab is from *Optitrack* company. You can think of it as GPS system for indoor environment. The software used to define rigid bodies (e.g. quadcopter) is called *Motive*