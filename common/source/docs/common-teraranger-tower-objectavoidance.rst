.. _common-teraranger-tower-objectavoidance:

=====================================
TeraRanger Tower for Object Avoidance
=====================================

[copywiki destination="copter"]

The `TeraRanger Tower <http://www.teraranger.com/teraranger-tower/>`__ can be used for Object Avoidance in Copter-3.5 and higher in Loiter and AltHold modes.  The sensor has a maximum useable range of about 4.5 meters.

..  youtube:: dOaO1mff9QM
    :width: 100%

Mounting the Sensor
===================

   .. image:: ../../../images/teraranger-tower.png
       :target: ../_images/teraranger-tower.png
       :width: 300px

The TeraRanger Tower should be mounted on the top of the vehicle so that sensors scans horizontally and its view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.

Connecting to the Pixhawk
=========================

  .. image:: ../../../images/teraranger-tower-serial.png
      :target: ../_images/teraranger-tower-serial.png
      :width: 300px

The image above shows how to connect TeraRanger Tower to Serial4 on Pixhawk.
Recommended supply current is 345mA at 12V (accepted voltage range is 10-20V).

Configuration through the Ground Station
========================================

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "921" if using Serial4.
- :ref:`PRX_TYPE <PRX_TYPE>` = "3" to enable the TeraRanger Tower (or 0 to disable).

Limitation
==========

The AC3.4 implementation has the following limitations.  Future versions will likely resolve these.

-  Object avoidance only works in Loiter mode and AltHold
-  The vehicle should stop before hitting objects but will never back away from objects that approach the vehicle (a slow backing away will be added in future firmware versions).

Please report issues found in the `support forums <http://discuss.ardupilot.org/c/arducopter/copter-3-5>`__ and we will try to address them.

Distance reporting
==================

The distance to the nearest object in 8 quadrants around the vehicle is recorded in the DataFlash log's PRX messages.

Real-time distances can be seen in the Mission Planner's proximity viewer which can be opened by moving to the MP's Flight Data screen, press Ctrl-F and push the Proximity button.