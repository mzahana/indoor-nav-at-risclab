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
Aafter you login, type **$$$** and hit **ENTER**

type
```sh
scan
```
to make sure that the device is operational. If there are networks, it should be listed.

### Serial setup
You can change the serial baudrate by
```sh
set u b 57600
```

### WiFi setup
Set authentification to WPA2-PSK only:
```sh
set wlan auth 3
```
set auto channel scan
```sh
set wlan channel 0
```
Tell the module to auto-join the network when powered on:
```sh
set wlan join 1
```
set  wireless name, SSID
```sh
set wlan ssid <your wifi ssid>
```
set WiFi password
```sh
set wlan phrase <password>
```
### IP setup
<div class="info">
Although you can use dynamic IPs, it is recommended to use static IP and disable the DHCP mode on your router, for more reliable and less latency communication.
</div>

This guide assumes UDP communication to a ground control station computer on IP 192.168.1.100, port 14550 (QGroundControl default port).
#### Set dynamic IP
Enable DHCP on each boot (for dynamic IP):
```sh
set ip dhcp 1
```
set IP protocol (UDP & TCP)
```sh
set ip protocol 3
```
Set remote port:
```sh
set ip remote 14550
```
set remote hos IP (IP of your PC):
```sh
set ip host 192.168.1.100
```
#### Test and save configurations
join the WiFi
```sh
join <WiFi ssid>
```
it it connects, it will show:
```sh
Asscoiated!
```
save and reboot
```sh
save
reboot
```
<div class="error">
Make sure that you save your settings, otherwise it will be lost
</div>

To check the settings current yon the device,
IP settings:
```sh
get ip
```
wifi settings:
```sh
get wlan
```
serial settings:
```sh
get u
```
#### Static IP
Disable DHCP mode
```sh
set ip dhcp 0
```
set the WiFi module's IP address
```sh
set ip address <choose ip>
```
your IP first 3 numbers (e.g. 192.168.1.\*) should bethe same as your router's first three numbers

set IP gateway (usually this is your router's IP). You can firdt set up dynamic IP, and then connect to the WiFi. Then, on the WiFi module command line type `get ip` to see the *gateway* and the *netmask*, and note them down. Set the *gateway* and *netmask* as follows,
```sh
set ip gateway <router ip address>
```
set *netmask*:
```sh
set ip netmask <netmask address>
```
set local port. You can leave the default (2000)
```sh
set ip localport 2000
```
set the remote host IP and remote port as before.

Save and reboot
```sh
save
reboot
```
Make sure that the device can join the WiFi netowrk. Log in to the dvice using 
#### Configure higher baud rates
<div class="error">
DO NOT set high baud rates while you are on serial (e.g. 921600), because you will not be able to log in again from the serial consol. You can set higher baud rate after you log in to the Wifly module via WiFi, using `telnet` command in Mac OS
</div>
To log in to the Wifly device via WiFi