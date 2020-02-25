.. _tricopter:

=======================
Tricopter Configuration
=======================

This page outlines the special settings required to get a TriCopter
flying.  The more :ref:`general instructions for setting up a multicopter <initial-setup>` should
be used for all other aspects of the setup.

.. image:: ../images/APM_2_5_MOTORS_TRI.jpg
    :target: ../_images/APM_2_5_MOTORS_TRI.jpg

Copter 3.5 (and higher)
=======================

As of Copter-3.5.x all of the multicopter firmware (quad, hexa, octa, octaquad, y6, tri, single, coax) including tricopter have been consolidated into a single firmware.  This means after loading the firmware please set:

- :ref:`FRAME_CLASS <FRAME_CLASS>` to 7 ("Tri")

Better separation of RC input and output parameters has also lead to some changes compared to earlier versions.  Here is a full list of tricopter specific parameters:

- :ref:`MOT_YAW_SV_ANGLE <MOT_YAW_SV_ANGLE>` : tail servo's maximum lean angle in degrees.  This allows for the rear motor's thrust to be adjusted appropriately depending upon the lean angle of the rear motor.  The default is 30 degrees.  "0" would mean the tail servo can only point directly up (which would not allow the vehicle to fly), "90" means the tail servo can point horizontally.
- :ref:`SERVO7_MIN <SERVO7_MIN>`: tail servo's lowest PWM value before binding occurs.
- :ref:`SERVO7_MAX <SERVO7_MAX>`: tail servo's highest PWM value before binding occurs.
- :ref:`SERVO7_TRIM <SERVO7_TRIM>`: tail servo's PWM value close to what is required to keep the tail from spinning.
- :ref:`SERVO7_REVERSED <SERVO7_REVERSED>`: tail servo's reverse setting.  0 = servo moves in default direction, 1 to reverse direction of movement.

The RC output channel used for the tail servo can be changed from it's default (channel 7) by setting the appropriate SERVOX_FUNCTION to 39.
For example the :ref:`Pixracer <common-pixracer-overview>` only has 6 output channels so the tail servo can be moved to output channel 5 by setting :ref:`SERVO5_FUNCTION <SERVO5_FUNCTION>` to 39.
Note that if the output channel is changed, the SERVOx_MIN, SERVOx_MAX, SERVOx_TRIM and SERVOx_REVERSED must be set appropriately for the new output channel.

Copter 3.4.x
============

- :ref:`MOT_YAW_SV_ANGLE <MOT_YAW_SV_ANGLE>` : tail servo's maximum lean angle in degrees.  This allows for the rear motor's thrust to be adjusted appropriately depending upon the lean angle.  The default is 30 degrees.  "0" would mean the tail servo can only point directly up (which would not allow the vehicle to fly), "90" means the tail servo can point horizontally.

The channel used for the tail servo can be changed from it's default (channel 7) by setting the appropriate SERVOx_FUNCTION to 39.  For example the :ref:`Pixracer <common-pixracer-overview>` only has 6 output channels so the tail servo can be moved to output channel 5 by setting :ref:`SERVO5_FUNCTION <SERVO5_FUNCTION>` to 39.

See Copter 3.3 section below for more parameters that can be adjusted.

Copter 3.3.x
============

-  ``MOT_YAW_SV_MIN <MOT_YAW_SV_MIN>``: tail servo's lowest PWM value before binding occurs.
-  ``MOT_YAW_SV_MAX <MOT_YAW_SV_MAX>``: tail servo's highest PWM value before binding occurs.
-  ``MOT_YAW_SV_TRIM <MOT_YAW_SV_TRIM>``: tail servo's PWM value close to what is required to keep the tail from spinning.
-  ``MOT_YAW_SV_REV <MOT_YAW_SV_REV>``: tail servo's reverse setting.  +1 = servo moves in default direction, -1 to reverse direction of movement.

Copter 3.2.1 (and earlier)
==========================

-  :ref:`RC7_MIN <RC7_MIN>`: tail servo's lowest PWM value before binding occurs.
-  :ref:`RC7_MAX <RC7_MAX>`: tail servo's highest PWM value before binding occurs.
-  :ref:`RC7_MIN <RC7_TRIM>`: tail servo's PWM value close to what is required to keep the tail from spinning.
-  :ref:`RC7_REVERSED <RC7_REVERSED>`: tail servo's reverse setting.  +1 = servo moves in default direction, -1 to reverse direction of movement.
