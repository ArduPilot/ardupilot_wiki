.. _tricopter:

=======================
Tricopter Configuration
=======================

This page outlines the special settings required to get a TriCopter
flying.  The more :ref:`general instructions for setting up a multicopter <initial-setup>` should
be used for all other aspects of the setup.

.. image:: ../images/APM_2_5_MOTORS_TRI.jpg
    :target: ../_images/APM_2_5_MOTORS_TRI.jpg

Copter 3.4 (and higher)
=======================

- :ref:`MOT_YAW_SV_ANGLE <MOT_YAW_SV_ANGLE>` : yaw servo's maximum lean angle in degrees.  This allows for the rear motor's thrust to be adjusted appropriately depending upon the lean angle.  The default is 30 degrees.  "0" would mean the rear servo can only point directly up (which would not allow the vehicle to fly), "90" means the rear servo can point horizontally.

The channel used for the tail servo can be changed from it's default (channel 7) by setting the appropriate RCX_FUNCTION to 39.  For example the :ref:`Pixracer <common-pixracer-overview>` only has 6 output channels so the tail servo can be moved to output channel 5 by setting :ref:`RC5_FUNCTION <RC5_FUNCTION>` to 39.

See Copter 3.3 section below for more parameters that can be adjusted.

Copter 3.3 (and higher)
=======================

-  :ref:`MOT_YAW_SV_MIN <MOT_YAW_SV_MIN>`: yaw servo's lowest PWM value before binding occurs.
-  :ref:`MOT_YAW_SV_MAX <MOT_YAW_SV_MAX>`: yaw servo's highest PWM value before binding occurs.
-  :ref:`MOT_YAW_SV_TRIM <MOT_YAW_SV_TRIM>`: yaw servo's PWM value close to what is required to keep the tail from spinning.
-  :ref:`MOT_YAW_SV_REV <MOT_YAW_SV_REV>`: yaw servo's reverse setting.  +1 = servo moves in default direction, -1 to reverse direction of movement.

Copter 3.2.1 (and earlier)
==========================

-  :ref:`RC7_MIN <RC7_MIN>`: yaw servo's lowest PWM value before binding occurs.
-  :ref:`RC7_MAX <RC7_MAX>`: yaw servo's highest PWM value before binding occurs.
-  :ref:`RC7_MIN <RC7_TRIM>`: yaw servo's PWM value close to what is required to keep the tail from spinning.
-  :ref:`RC7_MIN <RC7_REV>`: yaw servo's reverse setting.  +1 = servo moves in default direction, -1 to reverse direction of movement.
