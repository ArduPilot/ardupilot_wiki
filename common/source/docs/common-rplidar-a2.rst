.. _common-rplidar-a2:

===========================
RPLidar A2 360 degree lidar
===========================

The `RPLidar A2 <https://www.slamtec.com/en/Lidar/A2>`__ can be used for object avoidance in Copter-3.6 (and higher) and Rover-3.3 (and higher).  This page describes how to connect it directly to your autopilot.
See separate wiki pages on object avoidance for Copter and Rover for more details on how to setup the avoidance feature.

   .. image:: ../../../images/rplidar-a2.jpg
       :width: 300px

*image courtesy of robotshop.com*

Where to buy
------------

- Slamtec.com includes a `list of resellers here <https://www.slamtec.com/en/Home/Buy>`__

Specifications
--------------

- rotation rate: 10hz / 600 RPM
- sample rate: 4000 to 8000 samples/s
- range: 6m to 18m
- resolution: 0.9 degrees
- voltage/current requirement: 5V / 1.5A

Connecting and Configuring
--------------------------

   .. image:: ../../../images/rplidar-a2-pixhawk.jpg
       :target: ../_images/rplidar-a2-pixhawk.jpg
       :width: 600px

The lidar should be mounted horizontally on the top or bottom of the vehicle with the black cable pointing towards the rear of the vehicle.
Ensure the sensor's view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.

The lidar can be connected to the autopilot's serial input as shown above.
If using a Pixhawk/Pixhawk2 Telem1 (aka Serial1) should be used because it is more capable of providing the required 1.5A.

- :ref:`SERIAL1_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial1
- :ref:`SERIAL1_BAUD <SERIAL1_BAUD>` =  "115" if using Serial1
- :ref:`PRX_TYPE <PRX_TYPE>` = "5"
- :ref:`PRX_ORIENT <PRX_ORIENT>` = "0" if mounted on the top of the vehicle, "1" if mounted upside-down on the bottom of the vehicle.

It may be necessary to turn off flow control if using Telem1 (aka Serial1) or Telem2 (aka Serial2)

- :ref:`BRD_SER1_RTSCTS <BRD_SER1_RTSCTS>` =  "0" if using Serial1

More details on using this sensor for object avoidance on Copter can be found :ref:`here <common-object-avoidance-landing-page>`.
