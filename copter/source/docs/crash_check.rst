.. _crash_check:

===========
Crash Check
===========

Copter includes a crash check which disarms the motors in cases where the vehicle is likely out of control and has hit the ground.  This reduces damage to the vehicle and also reduces the chance of injury to people near the vehicle.

The crash check is similar to the :ref:`parachute release <parachute>` logic except that the parachute will normally release as the vehicle falls, while the crash check should trigger once the vehicle has hit the ground.

When will the crash check disarm the motors?
============================================
When all the following are true for 2 full seconds:

#. the vehicle is armed
#. the vehicle is not landed (as far as it can tell)
#. the current flight mode is *not* ACRO or FLIP
#. the vehicle is not accelerating by more than 3m/s/s
#. the actual lean angle has diverged from the desired lean angle (perhaps input by the pilot) by more than 30 degrees

What will happen when the crash check fires?
============================================

#. the motors will disarm
#. "Crash: Disarming" will be displayed on the Ground Station
#. a crash event will be written to the dataflash logs (look for EV, 12 in the logs)

How and when should the crash check be disabled?
================================================
In general the crash check should be left enabled but if the vehicle is likely to suffer from lean angle errors of over 30 degrees for a second or more it should be disabled.  This includes applications where the vehicle is being used to carry a guide wire from one mountain peak to another or being used for "drone boarding".

   .. image:: ../images/crash_check_drone_boarding.jpg
       :target: ../_images/crash_check_drone_boarding.jpg

Image courtesy of `The Verge <https://www.theverge.com/2016/2/3/10905970/droneboarding-is-happening>`__

In Copter-3.3.3 (and higher) the crash check can be disabled by setting :ref:`FS_CRASH_CHECK <FS_CRASH_CHECK>` to 0.

Below is a video describing the crash check from Copter-3.1 (the logic has changed in 3.3 and higher)

..  youtube:: xaw3-oSahtE
    :width: 100%
