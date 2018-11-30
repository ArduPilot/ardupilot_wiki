.. _common-lightware-sf40c-objectavoidance:

==========================
LightWare SF40/C 360 Lidar
==========================

The `Lightware SF40/C 360degree lidar <https://lightware.co.za/collections/lidar-rangefinders/products/sf40-c-100-m>`__ can be used for Object Avoidance in Copter-3.4 and higher in Loiter mode.

.. warning::

   This feature has not been tested in a wide variety of situations and should be used with caution.

..  youtube:: BDBSpR1Dw_8
    :width: 100%

Mounting the SF40c
------------------

   .. image:: ../../../images/lightware-sf40c.png
       :target: ../_images/lightware-sf40c.png
       :width: 300px

The SF40c should be mounted on the top or bottom of the vehicle so that the rotating portion scans horizontally and its view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.  The round gold and black lightware logo should be facing forward.
    
Connecting to the Flight Controller
-----------------------------------

   .. image:: ../../../images/lightware-sf40c-pixhawk.png
       :target: ../_images/lightware-sf40c-pixhawk.png

The diagram above shows how the SF40c can be connected to the flight controller's serial input.  The above pictures shows use of Serial4 but any free serial port can be used.

Configuration through the Ground Station
----------------------------------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "115" if using Serial4.
- :ref:`PRX_TYPE <PRX_TYPE>` = "1" to enable the SF40c (or 0 to disable).
- :ref:`PRX_ORIENT <PRX_ORIENT>` = "0" if mounted on the top of the vehicle, "1" if mounted upside-down on the bottom of the vehicle.
- :ref:`PRX_YAW_CORR <PRX_YAW_CORR>` allows adjusting the forward direction of the SF40c.  Even with the sensor's lightware logo pointed forwards, in testing the lidar has reported objects 20 ~ 30degrees off from their actual direction.  The best way to determine this value is to place an object in front of the sensor, connect a PC to the sensor using a USB cable, then open the Lightware Terminal application and enter "?TS,90,0" (search light, 90 degree wide beam directly forward).  The sensor should return two numbers, an angle and a distance.  The negative of the angle should be entered into the :ref:`PRX_YAW_CORR <PRX_YAW_CORR>` parameter.

More details on using this sensor for object avoidance on Copter can be found :ref:`here <copter-object-avoidance>`.
