.. _common-ark-dist-sr:

============
ARK DIST SR
============

[copywiki destination="copter,plane,rover,sub"]

The `ARK DIST SR <https://arkelectron.com/product/ark-dist-sr/>`__ ARK DIST SR is a low range, open source, DroneCAN, distance sensor. It has an approximate range of between 8cm to 30m.

.. image:: ../../../images/arkflow/ark_dist.jpg
   :target: ../_images/arkflow/ark_dist.jpg
   :width: 450px

Specifications
==============

-  **Sensors**

   - `Dronecan Small-Range Distance Sensor Module <https://dronecan.github.io/>`__
   - `Broadcom AFBR-S50LV85D Time-of-Flight Distance Sensor <https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors/afbr-s50lv85d>`__
  
    - Integrated 850 nm laser light source
    - Field-of-View (FoV) of 12.4° x 6.2° with 32 pixels
    - Typical distance range up to 30m
    - Operation of up to 200k Lux ambient light
    - Works well on all surface conditions
    - Transmitter beam of 2° x 2° to illuminate between 1 and 3 pixels
    - Reference Pixel for system health monitoring
  
-  **Connections**

   - Two Pixhawk Standard CAN Connectors
   
    - 4 Pin JST GH
   
   - Pixhawk Standard Debug Connector
    
    - 6 Pin JST SH

   - Pixhawk Standard UART Connector
    
    - 6 Pin JST SH
 
-  **Power Requirements**

   -  5V

    - 84mA Average
    - 86mA Max

-  **Other**

   - USA Built
   - FCC Compliant
   - 4 Pin Pixhawk Standard CAN Cable
   - LED Indicators
   - `ROS2 Support <https://github.com/ARK-Electronics/ros2_dronecan>`__

Where to Buy
------------

The sensor is available from `ARK Electronics <https://arkelectron.com/product/ark-dist-sr/>`__.

Connection to Autopilot
-----------------------

- The ARK DIST SR is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable.
- Multiple sensors can be connected by plugging additional sensors into the ARK DIST SR's second CAN connector.

Connection to Autopilot CAN:

- Set :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 24 (DroneCAN)
- Set :ref:`RNGFND1_MAX <RNGFND1_MAX>` = 30 to set range finder's maximum range to 30m

Connection to Autopilot with UART/MAVLink(SERIAL1 is shown as an example):

- Set :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 10 (MAVLink)
- Set :ref:`RNGFND1_MAX <RNGFND1_MAX>` = 30 to set range finder's maximum range to 30m
- Set :ref:`SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` = 1 (MAVLink) or the appropriate SERIALx_PROTOCOL for the selected serial port
- Set :ref:`SERIAL1_BAUD <SERIAL1_BAUD>` = 115 to set the baud rate to 115200 or the appropriate SERIALx_BAUD for the selected serial port
- Set :ref:`SERIALx_PROTOCOL <SERIALx_PROTOCOL>` = 2 (MAVLink2)

More Information
-----------------

* `ARK DIST SR <https://docs.arkelectron.com/sensor/ark-dist/ardupilot-instructions>`_

Testing and Setup
-----------------

See :ref:`common-rangefinder-setup`
