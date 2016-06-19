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