.. _common-teraranger-tower-objectavoidance:

================================
TeraRanger Tower/EVO (360 Lidar)
================================

The `TeraRanger Tower <https://www.terabee.com/portfolio-item/teraranger-tower-scanner-for-slam-and-collision-avoidance/>`__ can be used for Object Avoidance in Copter-3.5 and higher in Loiter and AltHold modes.  The sensor has a maximum useable range of about 4.5 meters.

..  youtube:: dOaO1mff9QM
    :width: 100%

Mounting the Sensor
-------------------

   .. image:: ../../../images/teraranger-tower.png
       :target: ../_images/teraranger-tower.png
       :width: 300px

The TeraRanger Tower should be mounted on the top of the vehicle so that sensors scans horizontally and its view is not obstructed by any portion of the vehicle including GPS mast, vehicle legs etc.

Connecting to the Autopilot
---------------------------

  .. image:: ../../../images/teraranger-tower-serial.png
      :target: ../_images/teraranger-tower-serial.png
      :width: 300px

The image above shows how to connect TeraRanger Tower to Serial4 on Pixhawk.
Recommended supply current is 345mA at 12V (accepted voltage range is 10-20V).

Configuration through the Ground Station
----------------------------------------

- :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` = "11" ("Lidar360") if using Serial4.
- :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` =  "921" if using Serial4.
- :ref:`PRX_TYPE <PRX_TYPE>` = "3" to enable the TeraRanger Tower (or 0 to disable).

More details on using this sensor for object avoidance on Copter can be found :ref:`here <common-object-avoidance-landing-page>`.
