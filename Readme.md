Battery Monitor with low energy Bluetooth readout
=================================================
Measure and display the 
- capacity in Ampere hours (Ah) of a battery
- the voltage of the battery (V)
- the current flowing from/to the battery (A)

of a camper/mobil home 12V battery on an Android Smartphone.

Prototype exists, but the schematic not completed yet.

![Prototype](images/prototype_w_sensor_2000px.jpg?raw=true&s=300px "Prototype of the Battery Monitor")

![Prototype close up](images/prototype_labeled_2000px.jpg?raw=true&s=300px "Prototype of the Battery Monitor (close up)")


Hardware
========
- ESP32 board
- Current sensor to measure the magnetic field 
- Step-down regulator 
- OpAmp

Current Sensor
==============
A split core current sensor with a Hall sensor is used to measure the current.

Advantage: sensor works contactless. No need to mess with the cabling, since the sensor is just placed around one of the battery cables.

Utilized model: YHDC HSTS016L +-20A
- 2.5+-0.625V (buffered by an OpAmp) 
- reference voltage 2.5V (buffered by an OpAmp) 

The reached resolution is about 50mA.

Analog to digital conversion
============================
Internal 12-bit ADC of the ESP32 processor. The ADC is quite noisy and an averaging (low pass filter) is used in the software 
in order to achieve the resolution of about 50mA.

Android APP
===========
Developed with the MIT App-Inventor2
- Low energy bluetooth (BLE) connection to the ESP32
- Main Screen with current, voltage, capacity
- Settings window 

![Battery Monitor APP main screen](images/app_main.png?raw=true&s=300px "Battery Monitor APP - main screen")

![Battery Monitor APP settings](images/app_settings.png?raw=true&s=300px "Battery Monitor APP - settings")


Software
========
The Arduino IDE is used for the code development.

- The BLE connection is initialized by the phone
- A sign-on message is expected by the ESP32. This is a hash value of the current time (salt) and a pre-shared pass phrase. 
- If the message is not received, the BLE connection is terminated.
- Data are send to the APP every second as a block of 20 bytes. The values are 16 bit integer values, which have been scaled to reflect the predefined number og significant digits. 

To Come
=======
Use ADS1115 16bit ADC board instead of internal ADC.

Advantage:
- less noise
- internal amplifier
