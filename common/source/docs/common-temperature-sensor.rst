.. _common_temperature_sensors:
[copywiki destination="plane,copter,rover,blimp"]

===================
Temperature Sensors
===================

ArduPilot supports up to nine individual temperature sensors with I2C interfaces:

- `TSYS01 <https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FTSYS01%7FA%7Fpdf%7FEnglish%7FENG_DS_TSYS01_A.pdf%7FG-NICO-018>`__
- `MCP9600 <https://ww1.microchip.com/downloads/en/DeviceDoc/MCP960X-Data-Sheet-20005426.pdf>`__
- `MAX31865 <file:///home/henry/Downloads/MAX31865.pdf>`__

.. image:: ../../../images/temperature-sensor.jpg
   :target: ../../_images/temperature-sensor.jpg

ArduPilot already has numerous possible sources for temperature reports: ESCs, Smart Batteries, Motor EFI and these independent sensors can be used to replace those devices temperature reports which already exist in ArduPilot. They can also just be logged.

Setup
=====

Attach sensor to one of the autopilot's I2C ports using SDA and SCL pins. Then set (examples shown for first sensor):

- :ref:`TEMP1_TYPE<TEMP1_TYPE>` = 0:Disabled,1:TYSYS01,2:MCP9600,3:MAX31865 Sensor type
- :ref:`TEMP1_BUS<TEMP1_BUS>` = 0-3 Board's I2C port on which the sensor is attached
- :ref:`TEMP1_ADDR<TEMP1_ADDR>` = the sensor's I2C address (0-127)
- :ref:`TEMP1_SRC<TEMP1_SRC>` = which other temperature reporting device type should have its temperature report replaced with this sensor's.
- :ref:`TEMP1_SRC_ID<TEMP1_SRC_ID>` = this identifies which individual device, of the above type, should have its report replaced.

Logging
=======

- :ref:`TEMP_LOG<TEMP_LOG>` = 0: No logging, 1: Log temperature reports from all temperature sensors
