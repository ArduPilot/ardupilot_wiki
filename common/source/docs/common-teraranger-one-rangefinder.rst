.. _common-teraranger-one-rangefinder:

==========================
TeraRanger One Rangefinder
==========================

The `TeraRanger One <http://www.teraranger.com/>`__ rangefinder is a
lightweight, high-performance distance measurement sensor based on
infrared Time-of-Flight (ToF) technology. It is much faster than
ultrasound and far smaller and lighter than laser-based systems. Its
small size, high performance and low power consumption make it ideal for
modern robotic applications, drone operations and automation.

More technical information about ToF sensors can be found
`here <https://www.terabee.com/portfolio-item/teraranger-tower-scanner-for-slam-and-collision-avoidance/>`__.

.. note::

   This rangefinder is only supported on the Pixhawk via I2C
   protocol. Only the TeraRanger One sensor is compatible with this
   protocol.

Connecting to the TeraRanger via I2C
====================================

The TeraRanger One sensor can be easily connected to the Pixhawk with
the `TeraRanger I2C adaptor <http://www.teraranger.com/product/adapters-for-oneduo/>`__.
Then it’s nearly a plug&play solution:

#. Solder the alimentation for the TeraRanger I2C adaptor:

   .. figure:: ../../../images/TeraRangerI2CAdapter-1.png
      :target: ../_images/TeraRangerI2CAdapter-1.png

      TR-I2CAD Power Cables

#. Connect the I2C Adaptor to the Pixhawk via a DF13 4S cable include
   into the Pixhawk box, and the TR-One to the I2C Adaptor:

   .. figure:: ../../../images/rangefinder_teraranger_one_pixhawk.jpg
      :target: ../_images/rangefinder_teraranger_one_pixhawk.jpg

      TR-One connection to Pixhawk with TR-I2CAD

Setup in Mission Planner
========================

To use the TeraRanger One sensor as rangefinder, connect with *Mission
Planner* and open the **Config/Tunin**\ g menu. Go into Full Parameter
List section and set:

-  ``RNGFND_MAX_CM``: 1400 (14m). This parameter sets the maximum
   distance that the TeraRanger One is used by the Pixhawk.
-  ``RNGFND_MIN_CM``: 20cm. This parameter set the minimum distance that
   the TeraRanger One is used by the Pixhawk.
-  ``RNGFND_TYPE``: 14

.. figure:: ../../../images/TeraRangerOne_MissionPlannerSettings.jpg
   :target: ../_images/TeraRangerOne_MissionPlannerSettings.jpg

   MissionPlanner Setup for using TR-One sensor on Pixhawk

Testing the sensor
==================

The distances read by the *TeraRanger One* sensor can be displayed on
the *Mission Planner Flight Data* menu. You have to double click on the
left bottom windows where quick values are displayed. Then choose
“sonarrange”.

.. figure:: ../../../images/TeraRangerOne_MissionPlannerEnableFlightData.jpg
   :target: ../_images/TeraRangerOne_MissionPlannerEnableFlightData.jpg

   Mission Planner: Setting to display rangefinder data on the Flight Datascreen

.. warning::

    When the Pixhawk boots, the TeraRanger One needs to already be powered on, or at 
    least be powered on at the same time as the Pixhawk. If you power the TeraRanger One 
    after the Pixhawk has booted, it will not be recognised!
