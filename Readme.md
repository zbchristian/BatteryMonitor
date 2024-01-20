Battery Monitor with low energy Bluetooth readout
=================================================
Measure and display the 
- capacity in Ampere hours (Ah) of a battery
- the voltage of the battery (V)
- the current flowing from/to the battery (A)

of a camper/mobil home 12V battery on an Android Smartphone.

The monitor is based on three modules (DC-DC converter, ESP32, ADS1115), which are placed either on a bread board or a PCB. 
The schematic and the layout of this base board can be found in the Eagle folder.

Depending on the used firmware, the device pulls about 10mA (25mA) from the 12V battery. An additional 8-9mA are drawn by the Hall current sensor.

![Prototype](images/BatteryMonitorADS1115_500px.jpg?raw=true "Prototype of the Battery Monitor")
![Prototype with Hall sensor](images/BatteryMonitorADS1115-Hall_500px.jpg?raw=true "Prototype of the Battery Monitor with split core current sensor")
![Prototype with shunt](images/BatteryMonitorADS1115-Shunt_500px.jpg?raw=true "Prototype of the Battery Monitor with shunt")



Hardware
========
* ESP32 board (Mini)
* Current sensor (Hall sensor to measure the magnetic field, or shunt resistor) 
* Step-down regulator (DC-DC Buck converter)
  * Input: 10V-20V
  * Output: 5.8V (adjustable version)
  * Max. current: 1A
* ADS1115 16 bit ADC

The 5.8V output voltage of the DC-DC converter is due to the fact, that the current sensor (Hall sensor) and the ADC require very stable 5V. 
This is provided by the 5V regulator MCP1702/1703. The ESP32 module has its own 3.3V regulator and can handle up to 6V on the Vcc pin. 

For a version utilizing the internal ADC, see the [Readme](Readme_internalADC.md).

Current Sensor
==============
A split core current sensor with a Hall sensor or a shunt resistor is used to measure the current.

Hall Sensor
-----------
Advantage: sensor works contactless. No need to mess with the cabling, since the sensor is just placed around one of the battery cables.

Utilized model: YHDC HSTS016L +-20A
- 2.5+-0.625V 
- reference voltage 2.5V

Shunt Resistor
--------------
A commercial shunt suitable for a high current (e.g. 100A) is placed into the ground connection to the battery. The voltage drop is small (e.g. 70mV for 100A).
The ADS1115 allows to change the voltage range down to +-256mV. This makes it suitable to measure the current with sufficient precision, without the need of an additional amplifier.

The reached resolution is for both cases about 50mA.

Analog to digital conversion
============================
The ADC ADS1115 with 16 bit resolution is readily available as a module. It exhibits 4 channels, a wide adjustable voltage range, a build in amplifier and can directly measure voltage differences.  

Android APP
===========
Developed with the MIT App-Inventor2
- Low energy bluetooth (BLE) connection to the ESP32
- Main Screen with current, voltage, capacity
- Settings window 


![Battery Monitor APP main screen](images/app_main.png?raw=true "Battery Monitor APP - main screen")
![Battery Monitor APP settings](images/app_settings.png "Battery Monitor APP - settings")


Software
========
The Arduino IDE is used for the code development. 
Three different versions of the ESP32 firmware exist:
- Utilize the internal ADC ([code](./BatteryMonitor_internal_ADC/BatteryMonitor_internal_ADC.ino))
- Utilize an ADS1115 external ADC ([code](./BatteryMonitor_ADS1115/BatteryMonitor_ADS1115.ino))
- UTilize an ADS1115 external ADC and reduce the power by going into the light sleep mode of the ESP32 The ([code](./BatteryMonitor_lowpower/BatteryMonitor_lowpower.ino))

Requires packages
-----------------
- ESP32 support with BLE and Wire
- ADS1115_WE by Wolfgang Ewald

Concept
-------
- Settings: define time intervals and used channels in the `#define` section of the code
- Setup: reduce CPU clock to save power, setup timers and the external ADC
- Loop: Read the battery voltage and current, calculate the Ah and power and store current battery state to flash
- The BLE connection is initialized by the phone
- A sign-on message is expected by the ESP32. This is a hash value of the current time (salt) and a pre-shared pass phrase. 
- If the message is not received, the BLE connection is terminated.
- For the first 60 seconds the connection is possible without the passphrase
- Data are send to the APP every second as a block of 20 bytes. The values are 16 bit integer values, which have been scaled to reflect the predefined number of significant digits. 

The versions differ mainly in the way the timing is handled. 

The first two versions utilize a 1msec timer interrupt to define time intervals and trigger time dependent 
actions (reading the voltage/current value, calculate battery capacity, transmit via BT). 

The third version handles the timing once per LOOP iteration. 
The LOOP is running once per 100msec and the corresponding delay is done in idle mode (no BT connection) by the light sleep mode of the ESP32. This reduces the 
required power substantially from about 25mA down below 10mA @ 12V, but requires, that the BT is disabled prior to entering the sleep mode. 
Therefore the BT connection is switched off for 5sec and switched on again for 1sec. As a consequence the connection initiated by a phone will take up to 5sec.

Status Printout of the Device
-----------------------------
An USB connection to the ESP32 allows to view the status print out of the device. Connect a terminal program to the corresponding virtual serial port (e.g. COM port).
The measured values and status infomrmations are printed to the terminal and allow to perform the calibration of the sensors.  


Calibration
===========
The calibration of the battery voltage and current measurement is required.

Battery Voltage
---------------
The measured voltage might vary due to resistor variations. The default voltage divider reduces the voltage by a factor of 0.25 ( 10k/(10k+30k) ). 
This factor is defined as `R12V`.
In addition a small voltage offset might be present. This should be measured with a good voltage meter and the corrections entered into the source code 
function `Vcal12`.

Current Sensor
--------------
A Hall sensor contains active components and the voltage to current conversion should be checked. An voltage offset leads to a none zero current displayed for 0 Amps.
The printout messages on a connected terminal allows to compare the measured values with the applied voltage/current. Enter the corrected values into the functions `VcalSensor` and `V2Amps`. 
Non linear dependencies might be entered into the latter function as well. 

Misc
====
On the PCB three additional IO pins are available on pin headers (IO2, IO4, IO5). These are not yet utilized and can be used to attach more hardware. The pin header for 
IO5 provides in addition 3.3V, which can be used to connect an active touch button (e.g. based on the TTP223 chip).  