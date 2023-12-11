.. _common-teraranger-tower-objectavoidance:

=======================================
TeraRanger Tower/ Tower EVO (360 Lidar)
=======================================

.. note:: The Tower is no longer available and has been replaced by the Tower EVO.

The `TeraRanger Tower <https://www.terabee.com/portfolio-item/teraranger-tower-scanner-for-slam-and-collision-avoidance/>`__ can be used for Object Avoidance in Loiter and AltHold modes.  The sensor has a maximum usable range of about 4.5 meters.

The `TeraRanger Tower EVO <https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower-evo/>`__ can be used for Object Avoidance in Loiter and AltHold modes.  The sensor has a maximum usable range of about 60 meters.

Mounting the Sensor and Connecting
----------------------------------

..  youtube:: 4tF-2uYU1KE
    :width: 100%

The TeraRanger Tower should be mounted on the top of the vehicle so that sensors scans horizontally and its view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.

Configuration through the Ground Station
----------------------------------------
Example setup below shown for first proximity sensor:

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "921" if using Serial4.
- :ref:`PRX1_TYPE <PRX1_TYPE>` = "3" to enable the TeraRanger Tower, = "6" for Tower EVO (or 0 to disable).
- :ref:`PRX1_ORIENT <PRX1_ORIENT>` = "0" Default.

More details on using sensors for object avoidance on Copter can be found :ref:`here <common-object-avoidance-landing-page>`.
