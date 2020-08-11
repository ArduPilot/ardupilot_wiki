.. _stabilize-mode:

==============
Stabilize Mode
==============

Stabilize mode allows you to fly your vehicle manually, but self-levels
the roll and pitch axis.

.. tip::

   If you're learning to fly, try :ref:`Alt Hold <altholdmode>` or
   :ref:`Loiter <loiter-mode>` instead of
   Stabilize. You'll have fewer crashes if you don't need to concentrate on
   too many controls at once.


.. warning::

   While stabilize mode does not necessarily require GPS, switching to RTL in case of emergency does. Make sure you do have a reliable
   position estimate prior to arming, most commonly provided by 3D GPS fix with sufficient HDOP.


Overview
========

-  Pilot's roll and pitch input control the lean angle of the copter. 
   When the pilot releases the roll and pitch sticks the vehicle
   automatically levels itself.
-  Pilot will need to regularly input roll and pitch commands to keep
   the vehicle in place as it is pushed around by the wind.
-  Pilot's yaw input controls the rate of change of the heading.  When
   the pilot releases the yaw stick the vehicle will maintain its
   current heading.
-  Pilot's throttle input controls the average motor speed meaning that
   constant adjustment of the throttle is required to maintain
   altitude.  If the pilot puts the throttle completely down the motors
   will go to their minimum rate (MOT_SPIN_ARMED) and if the vehicle
   is flying it will lose attitude control and tumble. However, this behavior can be changed
   by enabling :ref:`airmode` 
-  The throttle sent to the motors is automatically adjusted based on
   the tilt angle of the vehicle (i.e. increased as the vehicle tilts
   over more) to reduce the compensation the pilot must do as the
   vehicle's attitude changes.

.. note::

   Always switch into a manual mode such as stabilize if the
   autopilot fails to control the vehicle. Maintaining control of your
   copter is your responsibility.

AirMode
=======

Stabilize mode can be setup to provide full stabilization at idle throttle. See :ref:`airmode` 

.. _stabilize-mode_tuning:

Tuning
======

See :ref:`common-tuning` for complete tuning topics.

Using :ref:`AutoTune <autotune>` may allow you to automatically determine the best Stabilize and Rate PID values. It is highly suggested running AutoTune on your vehicle rather
than manually adjusting PIDs. However, see :ref:`ac_rollpitchtuning` for roll and pitch manual tuning.

Other important parameters
--------------------------
-  :ref:`ANGLE_MAX<ANGLE_MAX>` controls the maximum lean angle which by default is 4500
   (i.e. 45 degrees)
-  ANGLE_RATE_MAX controls the maximum requested rotation rate in the
   roll and pitch aixs which by default is 18000 (180deg/sec).
-  :ref:`ACRO_YAW_P <ACRO_YAW_P>` controls how quickly copter rotates based on a pilot's
   yaw input.  The default of 4.5 commands a 200 deg/sec rate of
   rotation when the yaw stick is held fully left or right.  Higher
   values will make it rotate more quickly.


