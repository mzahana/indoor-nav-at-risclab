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

## 1- Motion capture setup
Motion capture (or Mocap in short) is used to provide accurate positions and orientations in an indoor environment. The mocap setup we have in the lab is from *Optitrack* company. You can think of it as GPS system for indoor environment. 

Mocap mainly consists of cameras, network switches, and a PC with a special software. Cameras capture images which contain special *reflective markers*. Those markers are used to track objects (rigid bodies) they are attached to. Then, images from all cameras are transmitted to the PC software (called *Motive*) through the network switches, in order to do further image processing. *Motive* extracts useful information about captured rigid bodies such as position and orientation. Such information can be further transmitted through network to other PCs for further usage. Rigid bodies are defined by at least 3 reflective markers that are rigidly mounted on the object of interest.

For this tutorial, it is assumed that the Mocap is already calibrated.

What we need in this tutorial is to
* open *Motive* project
* define rigid bodies
* configure streaming parameters in Motive
* use the Streaming Application to send mocap info to Pixhawk


**Follow the following steps in order.**

* Open *Motive* software, and choose 'Open Existing Project'. Choose a recent project that represents the latest calibration settings.
![](startup screen.PNG)
![](choose project.PNG)

* close the *Camera Preview* view, and leave the *Perspective View* view for 3D viewing of objects.
 ![](motive1.PNG)
* place the object in the cage (e.g. quadcopter) with mounted markers (minimum 3 markers).
* select markers in the *Perspective View* and create a rigid body
![](createRigidbody.png)
You can know your rigid body number from the *Rigid Body*, after you select the rigid body in the *Perspective View*

* Now, activate streaming over network as follows
 ![](motiveStreamTab.PNG)
* Connect the wireless serial module to the Mocap PC (e.g. XBee)
* Open Mocap streaming App.
![](mocapstream.PNG)
* Select the proper *Vehicle ID*
* In the *Serial Connection* tab, select the proper serial port of the communication module from the *Port Name* drop menu. Set the *Baud Rate* to `57600`. Finally, click the *Connect* button. If the connection is successful, it will show a status message in the *Port Status* field.
* In the *Mocap Connection* tab, leave the *Mocap IP* and *Clien IP* to the defaults IPs (`127.0.0.1`). Hit the *Connect* button.
* If the connection is successful, you should see the defined rigid bodies in the *Received Bodies* list box.
* Select the one corresponds to the quadcopter. Then, check the *stream to Mav* checkbox.
* Now, your quad should be getting its position and orientaion feedback from the Mocap system.

## 2-Quadcopter setup
This tutorial assumes that the quadcopter is setup and equipped with a calibrated Pixhawk (or Pixracer) flight controller.

In this Demo, the quadcopter is assumed to have an ODROID on-board, two serial communication modules (e.g. XBee). One for the Mocap connection, and the other for MATLAB connection.

## 3-ODROID setup
In this Demo, ODROID is used to capture real-time images and stream them over WiFi network to a MATLAB session. The streaming application is assumed to be installed on ODROID and ready to be used. Also, the ODROID is assumed to be setup to connect to a local WiFi network.

Check [this guide](https://github.com/mzahana/Image_Live_Stream) to see how to install the streaming app on ODROID.

To run the application, follow the following steps in order,
* connect a compatible camera to ODROID
* connect a compatibe WiFi module to ODROID (use the ODROID WiFi adapter)
* power on the ODROID
* from your laptop (which is connected to the same local WiFi network as the ODROID), open a terminal and remotely log-in to ODROID

```sh
$ ssh odroid@192.168.1.113
# password: odroid
```
`odroid` is the user account name. `192.168.1.113` is the ODROID's IP address.
* navigate to the app folder and run it

```sh
$ cd ~/Desktop/imgstream/Image_Live_Stream/opencv_stream/stream_cpp
$$ ./sender 192.168.1.112 10000
```
`192.168.1.112` is your machine's IP address. `10000` is the port that is going to be opened in your MATLAB. You can choose whatever another port, but make sure it matches the one used in your MATLAB.
* now, the ODROID is sending image to the specified IP and port.

## 4-MATLAB setup
In this Demo, MATLAB is used to communicate with Pixhawk (or Pixracer) and receive live-stream of images from ODROID.

<div class="warning">
NOTE: You need to use the MATLAB files associated with this Demo. Please ask for your free copy.
</div>

We are going to use two main MATLAB classes in this Demo. One is called **MatMav**, and the other is called **ImgStream**.

**MatMav** is a MATLAB class that is used to communicate with Pixhawk. **ImgStream** is a MATLAB class that is use to receive live image stream from ODROID (or any Linux computer) over network.

Before you use the MATLAB files associated with this demo, you should setup your enviornment properly. 
<div class="warning">
NOTE: Before you use the MATLAB files associated with this demo, you should setup your enviornment properly. Namely, you need to associate your MATLAB with a C/C++ compiler, and instal OpenCV.
</div>
Please follow the OpenCV installation as follows,
* for Mac OS ([see this video](https://www.youtube.com/watch?v=U49CVY8yOxw))
* for Windows, ([see this video](https://www.youtube.com/watch?v=EcFtefHEEII))

Google how to associate your MATLAB with a comiler.

* Download the MATLAB folder associated with this Demo.
* Open MATLAB and navigate to that folder.
* run the `setup.m` file.

If all goes well, you should get the message `Setup is done`
* check the `Demo1` file to get familiar with *MatMav* and *ImgStream* classes.

**Good Luck!**