# Recommended WiFi setup
---

In this tutorial, we are going to use the *ESP8266* WiFi module to communicate with *Pixhawk* via Wifi.

Required:
* [ESP-07 ESP8266 Serial Wi-Fi Wireless Transceiver Module](http://www.dx.com/p/esp-07-esp8266-serial-wi-f-wireless-module-w-built-in-antenna-compatible-with-3-3v-5v-for-arduino-400559#.V0rfbGMwzww)
* [FTDI/USB cable](http://www.robotshop.com/en/ftdi-usb-to-ttl-serial-cable-5v.html): to flash firmware

* Connect the FTDI/USB cable to the ESP module. The orange cable (TX) is connected to (RX) pin on the module. Yellow cable (RX) is connected to (TX) pin on the module. Connect the power (red) and ground (black).
* Follow the guide in the link below to flash the *mavesp8266* firmware.

<div class="info">
Follow the
<a href="https://pixhawk.org/peripherals/8266">this guide</a>
to setup the ESP8266.
</div>

**NOTE**: Use `platformio run -e esp01_1m -t upload` to upload the firmware to the board.


