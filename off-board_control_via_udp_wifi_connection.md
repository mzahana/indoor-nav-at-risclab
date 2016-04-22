# Off-board WiFi interface



---
In this section, we learn how to communicate with *Pixhawk* via WiFi.

Required:
* *Pixhawk*: calibrated and ready to fly
* WiFi module: [RN-XV WiFly Module - Wire Antenna](https://www.sparkfun.com/products/10822)
* [XBee explorer USB](https://www.sparkfun.com/products/11812): to configure WiFi module via PC
* [XBee explorer regulated](https://www.sparkfun.com/products/11373): to interface with *Pixhawk*
* *MatMav* toolbox: to run offboard control from *MATLAB*

<div class="info">
Tip: To obtain MatMav, contact matmav.toolbox@gmail.com 
<div>

In this tutorial, *TELEM2* is going to be used to connect the WiFi module. However, *TELEM1* can be used too, but will require further configuration steps.

{% hint style='tip' %}
Tip: It is recommended to set the baud rate of TELEM2 to 921600
{% endhint %}