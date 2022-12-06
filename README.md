# ESP8266_MQTT_WaterMeter

This is a personal project that usese a hall effect sensor to detect the stator spinning in my water meter to figure out how much water we're using.
Send the updates to HomeAssistant via MQTT. 

Uses a hall effect sensor, 3pin package. Pretty sure it's a Honeywell SS494B, but the one you'll need is likely somewhat specific to your meter and the strength of the magnetic field it puts out.
I'm using an ADS1115 16 bit Analog to digital converter on a breakout board. You really need all those bits.

This package uses PlatformIO.

More details in the main.cpp

Wiring:

![IMG_5063](https://user-images.githubusercontent.com/787708/205804021-505c545b-1318-43aa-a8ae-b4ac05252396.JPEG)

When i get it off the breadboard I'll make some better notes, but nothing too tricky here. I2C communication to the '8266, analog in off the sensor to A0. Voltages and grounds all around. 5V reference to the sensor, but 3.3 to power the ADC. 
