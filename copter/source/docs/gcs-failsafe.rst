.. _gcs-failsafe:

============
GCS Failsafe
============

This page covers the set-up and testing of the Ground Station Control
(GCS) failsafe.

.. note::

   Copter also supports other failsafes: :ref:`Radio <radio-failsafe>`,
   :ref:`Battery <failsafe-battery>`, and :ref:`EKF / DCM Check <ekf-inav-failsafe>`.

Overview
========

The Ground Station Control (GCS) failsafe controls how Copter will
behave if contact with the GCS is lost. Depending on the setting and
vehicle position the failsafe will either RTL/LAND, or continue an
active mission.

More specifically, if you have been using the GCS to control Copter
(i.e. using a joystick) and then lose GCS contact for at least 5 seconds
the following will happen:

-  Disarm motors - if you are in stabilize or acro mode and your
   throttle is zero
-  RTL - if you have a GPS lock and are more than 2 meters from your
   home position
-  LAND - if you have no GPS lock or are within 2 meters of home
-  Continue with the mission - if you are in AUTO mode and have set the
   GCS Failsafe Options to 2 (Enabled_continue_in_auto_mode).

If the failsafe clears (contact with the ground station is restored)
Copter will remain in it's current flight mode. It
will **not** automatically return to the flight mode that was active
before the failsafe was triggered. This means that if, for example, you
are flying in stabilize mode when the failsafe is triggered, and this
causes the flight mode to change to RTL or LAND, if you wish to re-take
control of the copter you will need to set your flight mode again back
to stabilize.

Setting the failsafe
====================

In Mission Planner's Advanced Parameter List, set the
:ref:`FS_GCS_ENABLE <FS_GCS_ENABLE>` parameter to:

-  0 to disable the GCS failsafe
-  1 to enable and always RTL in event of loss of contact
-  2 to RTL unless in AUTO mode in which case we should continue with
   the mission

.. image:: ../images/FailsafeAdvPar801.jpg
    :target: ../_images/FailsafeAdvPar801.jpg

.. note::

   All Failsafe Parameters can be observed or set from the Advanced
   Parameter List.
