.. _archived-tricopter:

**ARCHIVED SETUP INSTRUCTIONS PRIOR TO COPTER 3.6 VERSIONS**

Copter 3.4.x
============

- :ref:`MOT_YAW_SV_ANGLE <MOT_YAW_SV_ANGLE>` : tail servo's maximum lean angle in degrees.  This allows for the rear motor's thrust to be adjusted appropriately depending upon the lean angle.  The default is 30 degrees.  "0" would mean the tail servo can only point directly up (which would not allow the vehicle to fly), "90" means the tail servo can point horizontally.

The channel used for the tail servo can be changed from its default (channel 7) by setting the appropriate SERVOx_FUNCTION to 39.  For example the :ref:`Pixracer <common-pixracer-overview>` only has 6 output channels so the tail servo can be moved to output channel 5 by setting :ref:`SERVO5_FUNCTION <SERVO5_FUNCTION>` to 39.

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
