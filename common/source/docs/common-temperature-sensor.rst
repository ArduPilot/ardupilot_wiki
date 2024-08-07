.. _common_temperature_sensors:
[copywiki destination="plane,copter,rover"]

===================
Temperature Sensors
===================

ArduPilot supports up to nine individual temperature sensors with I2C interfaces:

- `MAX31865 <https://www.analog.com/media/en/technical-documentation/data-sheets/MAX31865.pdf>`__
- `MCP9600 <https://ww1.microchip.com/downloads/en/DeviceDoc/MCP960X-Data-Sheet-20005426.pdf>`__
- `TSYS01 <https://www.te.com/usa-en/product-G-NICO-018.datasheet.pdf>`__
- `TSYS03 <https://www.te.com/usa-en/product-CAT-DTS0001.datasheet.pdf>`__

.. image:: ../../../images/temperature-sensor.jpg
   :target: ../_images/temperature-sensor.jpg

In addition, analog temperature monitors and DroneCAN based sensors can be used.

ArduPilot already has numerous possible sources for temperature reports: ESCs, Smart Batteries, Motor EFI and these independent sensors can be used to replace those devices temperature reports which already exist in ArduPilot. They can also just be logged.

I2C Sensor Setup
================

Attach sensor to one of the autopilot's I2C ports using SDA and SCL pins. Then set (examples shown for first sensor):

- :ref:`TEMP1_TYPE<TEMP1_TYPE>` = 0:Disabled, 1:TSYS01, 2:MCP9600, 3:MAX31865, 4: TSYS03 Sensor type
- :ref:`TEMP1_BUS<TEMP1_BUS>` = 0-3 Board's I2C port on which the sensor is attached
- :ref:`TEMP1_ADDR<TEMP1_ADDR>` = the sensor's I2C address (0-127)
- :ref:`TEMP1_SRC<TEMP1_SRC>` = which other temperature reporting device type should have its temperature report replaced with this sensor's.
- :ref:`TEMP1_SRC_ID<TEMP1_SRC_ID>` = this identifies which individual device, of the above type, should have its report replaced.

Analog Sensor Setup
===================

Atttch the sensor to an analog input pin. The pin number can be determined from the autopilot's wiki page. Then set (examples shown for first sensor):

- :ref:`TEMP1_TYPE<TEMP1_TYPE>` = 5 : Analog
- :ref:`TEMP1_PIN<TEMP1_PIN>` = the analog pin number (for example: 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1, etc.)

The output voltage vs temperature curve of the sensor can be adjusted to match its characteristic via an up to 4th order polynomial (deg = a0 + a1\*voltage + a2\*voltage^2 + a3\*voltage^3 + a4\*voltage^4) : 

- :ref:`TEMP1_A0<TEMP1_A0>` = a0 
- :ref:`TEMP1_A1<TEMP1_A1>` = a1
- :ref:`TEMP1_A2<TEMP1_A2>` = a2
- :ref:`TEMP1_A3<TEMP1_A3>` = a3
- :ref:`TEMP1_A4<TEMP1_A4>` = a4

DroneCAN Setup
==============

Make sure the CAN port is setup. (example below for first CAN port using first driver):

- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1 (First driver)
- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1 (DroneCAN)

Then set (examples shown for first sensor):

- :ref:`TEMP1_TYPE<TEMP1_TYPE>` = 6 (DroneCAN)

Logging
=======

- :ref:`TEMP_LOG<TEMP_LOG>` = 0: No logging, 1: Log temperature reports from all temperature sensors
