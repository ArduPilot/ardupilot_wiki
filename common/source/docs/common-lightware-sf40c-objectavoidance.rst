.. _common-lightware-sf40c-objectavoidance:

====================================
LightWare SF40c for Object Avoidance
====================================

[copywiki destination="copter"]

The `Lightware SF40c 360degree lidar <http://www.lightware.co.za/shop/en/scanning-and-obstacle-detection/45-sf40c-100-m.html>`__ can be used for Object Avoidance in Copter-3.4 and higher in Loiter mode.

.. warning::

   This feature is new for Copter-3.4 and has not been tested in a wide variety of situations.  It should be used with caution.

..  youtube:: BDBSpR1Dw_8
    :width: 100%

Mounting the SF40c
==================

   .. image:: ../../../images/lightware-sf40c.png
       :target: ../_images/lightware-sf40c.png
       :width: 300px

The SF40c should be mounted on the top or bottom of the vehicle so that the rotating portion scans horizontally and its view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.  The round gold and black lightware logo should be facing forward.
    
Connecting to the Pixhawk
=========================

   .. image:: ../../../images/lightware-sf40c-pixhawk.png
       :target: ../_images/lightware-sf40c-pixhawk.png

The diagram above shows how the SF40c can be connected to the flight controller's serial input.  The above pictures shows use of Serial4 but any free serial port can be used.

Configuration through the Ground Station
========================================

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "115" if using Serial4.
- :ref:`PRX_TYPE <PRX_TYPE>` = "1" to enable the SF40c (or 0 to disable).
- :ref:`PRX_ORIENT <PRX_ORIENT>` = "0" if mounted on the top of the vehicle, "1" if mounted upside-down on the bottom of the vehicle.
- :ref:`PRX_YAW_CORR <PRX_YAW_CORR>` allows adjusting the forward direction of the SF40c.  Even with the sensor's lightware logo pointed forwards, in testing the lidar has reported objects 20 ~ 30degrees off from their actual direction.  The best way to determine this value is to place an object in front of the sensor, connect a PC to the sensor using a USB cable, then open the Lightware Terminal application and enter "?TS,90,0" (search light, 90 degree wide beam directly forward).  The sensor should return two numbers, an angle and a distance.  The negative of the angle should be entered into the :ref:`PRX_YAW_CORR <PRX_YAW_CORR>` parameter.

Limitation
==========

The AC3.4 implementation has the following limitations.  Future versions will likely resolve these.

-  Object avoidance only works in Loiter and AltHold modes.
-  The vehicle should stop before hitting objects but will never back away from objects that approach the vehicle (a slow backing away will be added in future firmware versions).

Please report issues found in the `support forums <http://discuss.ardupilot.org/c/arducopter/copter34>`__ and we will try to address them.

DataFlash logging
=================

The distance to the nearest object in 8 quadrants around the vehicle is recorded in the DataFlash log's PRX messages.

Real-time distances can be seen in the Mission Planner's proximity viewer which can be opened by moving to the MP's Flight Data screen, press Ctrl-F and push the Proximity button.
