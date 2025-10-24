.. _common-rplidar-a2:

=================================================
RPLidar A1, A2, A2M12, C1 and S1 360 degree lidar
=================================================

The `RPLidar C1 <https://www.slamtec.com/en/C1>`__, `RPLidar A1 <https://www.slamtec.com/en/Lidar/A1>`__, `RPLidar A2 <https://www.slamtec.com/en/Lidar/A2>`__, RPLidar A2M12, or RPlidar S1 can be used for object avoidance in ArduPilot.  This page describes how to connect them directly to your autopilot.
See separate wiki pages on object avoidance for Copter and Rover for more details on how to setup the avoidance feature.

   .. image:: ../../../images/rplidar-a2.jpg
       :width: 300px

   .. image:: ../../../images/rplidar-s2.jpg
       :width: 300px

   .. image:: ../../../images/rplidar-c1.jpg
       :width: 300px

*image courtesy of robotshop.com*

Where to buy
------------

- Slamtec.com includes a `list of resellers here <https://www.slamtec.com/en/Home/Buy>`__

Connecting and Configuring
--------------------------

   .. image:: ../../../images/rplidar-a2-pixhawk.jpg
       :target: ../_images/rplidar-a2-pixhawk.jpg
       :width: 600px

   .. image:: ../../../images/rplidar-s2-cube.png
       :target: ../_images/rplidar-s2-cube.png
       :width: 600px

The lidar should be mounted horizontally on the top or bottom of the vehicle with the black cable pointing towards the rear of the vehicle.
Ensure the sensor's view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.

The lidar can be connected to the autopilot's serial input as shown above.
Be sure that the autopilot's 5V supply is capable of supplying the unit's required current. Otherwise provide an independent 5V supply to the unit.

Example setup below shown for first proximity sensor using SERIAL1 UART:

- :ref:`SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` = "11" ("Lidar360")
- :ref:`SERIAL1_BAUD <SERIAL1_BAUD>` = "460800" for C1, "115" for A2, "256" for S1, A2M12
- :ref:`PRX1_TYPE <PRX1_TYPE>` = "5"
- :ref:`PRX1_ORIENT <PRX1_ORIENT>` = "0" if mounted on the top of the vehicle, "1" if mounted upside-down on the bottom of the vehicle.

It may be necessary to turn off flow control if using Telem1 (aka Serial1) or Telem2 (aka Serial2)

- :ref:`BRD_SER1_RTSCTS <BRD_SER1_RTSCTS>` =  "0" if using Serial1
- :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` =  "0" if using Serial2

More details on using this sensor for object avoidance on Copter can be found :ref:`here <common-object-avoidance-landing-page>`.
