.. _crash_check:

===========
Crash Check
===========

Sub includes a crash check which can disarm the motors in cases where the vehicle is likely out of control and has hit something.  This reduces damage to the vehicle and its surroundings, and also reduces the chance of injury to people or creatures nearby.

When will the crash check disarm the motors?
============================================
When all the following are true for 2 full seconds:

* The :ref:`FS_CRASH_CHECK<FS_CRASH_CHECK>` = 2 ( A value of "0" disables the check while "1" will send only a warning to the GCS)
* The vehicle is armed
* The current flight mode is *not* ACRO or MANUAL
* The actual lean angle has diverged from the desired lean angle (perhaps input by the pilot) by more than 30 degrees.

What will happen when the crash check fires?
============================================

* The motors will disarm
* "Crash: Disarming" will be displayed on the Ground Station
* A crash event will be written to the dataflash logs (look for EV, 12 in the logs)

How and when should the crash check be disabled?
================================================
In general the crash check should be left enabled but if the vehicle is likely to suffer from lean angle errors of over 30 degrees for a second or more it should be disabled.  