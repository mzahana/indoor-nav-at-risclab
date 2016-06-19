# Image live stream from ODROID to MATLAB

Required:
* ODROID with OpenCV installed.
* [Mac OS with OpenCV installed in default locations.](https://www.youtube.com/watch?v=U49CVY8yOxw)
* MATLAB with associated compiler e.g. XCode
* WiFi network (Access Point)
* [Streaming ODROID application + MATLAB receiving application](https://github.com/mzahana/Image_Live_Stream)

## ODROID setup
* setup OpenCV
* setup streaming application
### setup OpenCV
* Make sure that your odroid is connected to internet.
* Open a terminal window, and run the follwoing command,
```sh
$ sudo apt-get -y install libopencv-dev
$ sudo apt-get -y install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
### setup streaming app
* create a clean directory and navigate to it e.g.
```sh
$ cd ~/Desktop
$ mkdir imgstream
$ cd imgstream
```
* clone the streaming app from Github
```sh
$ git clone https://github.com/mzahana/Image_Live_Stream.git
$ cd Image_Live_Stream
```
* navigate to the `stream_cpp` folder, and compile the app
```sh
$ cd opencv_stream/stream_cpp
$ cmake . & make
```
if all goes well, then two executable files should be generated: `sender` and `receiver`. Otherwise, make sure that you installed OpenCV properly in the default locations.

* To stream images over network, use the `sender` app after you connect a camera to ODROID. To use the `sender` app, use the following command in a terminal, inside the `stream_cpp` folder,
```sh
$ ./sender 192.168.1.100 10000
```
where `192.168.1.100` is the IP of machine running MATLAB (the host machine) (which should be on the same network as the ODROID's). `10000` is the port that MATLAB is listening on. Use appropriate IP and port that match the host ones.

## MATLAB setup
* Make sure that you installed XCode on your Mac OS (Google it).
* Make sure that you associat your MATLAB with XCode compiler (Google it).
* Navigate to the `Image_Live_Stream` folder that you  downloaded from Github.

run the `setup.m` file
```matlab
>> setup
```

If all goes well, you are ready to receive live stream of images from ODROID.
* Look at the `testScript.m` file to see how you can use the *ImgStream* class to establish the connection, and receive image data.