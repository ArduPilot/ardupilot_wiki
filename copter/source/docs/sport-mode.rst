.. _sport-mode:

==========
Sport Mode
==========

Sport Mode is also known as "rate controlled stabilize" plus Altitude
Hold.

.. note:: this mode is not included by default in the firmware built and available on the `Firmware Server <https://firmware.ardupilot.org>`__ . Either the user must build his own firmware with this mode enabled or use the `Custom Firmware Server <https://custom.ardupilot.org>`__

Overview
========

-  It was designed to be useful for flying FPV and filming `dolly shots <https://en.wikipedia.org/wiki/Dolly_shot>`__ or fly bys because
   you can set the vehicle at a particular angle and it will maintain
   that angle.
-  The pilot's roll, pitch and yaw sticks control the rate of rotation
   of the vehicle so when the sticks are released the vehicle will
   remain in its current attitude.
-  The vehicle will not lean more than 30 degrees (this angle is
   adjustable with the :ref:`ATC_ANGLE_MAX<ATC_ANGLE_MAX>` parameter)
-  The altitude is maintained with the altitude hold controller so the
   vehicle will attempt to hold its current altitude when the sticks
   are within the mid-throttle deadzone, which is adjustable with
   :ref:`THR_DZ<THR_DZ>`. It will climb or descend at up
   to 2.5m/s (this speed is adjustable with the :ref:`PILOT_SPD_UP<PILOT_SPD_UP>` and :ref:`PILOT_SPD_DN<PILOT_SPD_DN>`
   parameters). The acceleration used to establish these speeds is set by :ref:`PILOT_ACC_Z<PILOT_ACC_Z>`.
