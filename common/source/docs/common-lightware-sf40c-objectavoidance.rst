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
    
Connecting to the Autopilot
---------------------------

   .. image:: ../../../images/lightware-sf40c-pixhawk.png
       :target: ../_images/lightware-sf40c-pixhawk.png

The diagram above shows how the SF40c can be connected to the autopilot's serial input.  The above pictures shows use of Serial4 but any free serial port can be used.

Configuration through the Ground Station
----------------------------------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "921" if using Serial4.
- :ref:`PRX_TYPE <PRX_TYPE>` = "7" (LightwareSF40c) or "1" (LightwareSF40C-legacy) if using a very old version of the sensor
- :ref:`PRX_ORIENT <PRX_ORIENT>` = "0" if mounted on the top of the vehicle, "1" if mounted upside-down on the bottom of the vehicle.
- :ref:`PRX_YAW_CORR <PRX_YAW_CORR>` allows adjusting the forward direction of the SF40c.  One way to determine this angle is to use the Mission Planner's Setup >> Advanced, Proximity viewer and then walk around the vehicle and ensure that the sector distances shorten appropriately.
- :ref:`PRX_IGN_ANG1 <PRX_IGN_ANG1>` and :ref:`PRX_IGN_WID1 <PRX_IGN_WID1>` parameters allow defining zones around the vehicle that should be ignored.  For example to avoid a 20deg area behind the vehicle, set :ref:`PRX_IGN_ANG1 <PRX_IGN_ANG1>` to 180 and :ref:`PRX_IGN_WID1 <PRX_IGN_WID1>` to 10.

More details on using this sensor for object avoidance on Copter can be found :ref:`here <common-object-avoidance-landing-page>`.
