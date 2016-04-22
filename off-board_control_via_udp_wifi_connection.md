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
To obtain MatMav, contact matmav.toolbox@gmail.com 
</div>

In this tutorial, *TELEM2* is going to be used to connect the WiFi module. However, *TELEM1* can be used too, but will require further configuration steps.

<div class="info">
It is recommended to set the baud rate of TELEM2 to 921600 for faster data exchange, and less latency.
</div>

## Pixhawk TELEM setup
To set the baude rate of *TELEM2* to 921600, connect *Pixhawk* to *Qgroundcontrol*. Go to the *System* tab. Change the *SYS_COMP* parameter to use companion with 921600 baudrate. Restart *Pixhawk* to take effect.

## WiFi module setup

Official Roving Network documentation
[Reference Manual](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Wireless/WiFi/WiFly-RN-UM.pdf)

Connect the WiFi module to the XBee explorer USB board and connect it to the computer. You will need to use a serial terminal. For Mac, use the Mac terminal. For Windows it is recommended to use *TerTerm*.

On a Mac terminal, use the screen command to log into the Wifly

```sh
screen /dev/tty.usbserial-FTFABC 9600 8N1
```
`/dev/tty.usbserial-FTFABC` is the device port on Mac. You can find yours using
```sh
ls /dev/tty*
```
