.. _set-motor-range:

===================
Setting Motor Range
===================

Most ESCs have a dead zone at the bottom of their range.  This page outlines how to test the size of the range and then set the spin-when-armed and min throttle values appropriately.

Although not required, advanced users may wish to take the next step by measuring and adjusting the :ref:`motor thrust curve <motor-thrust-scaling>`.

.. note::

   Please complete the :ref:`ESC calibration <esc-calibration>` before setting the motor range

Measuring the deadzone
======================

-  Remove the propellers from the vehicle
-  Connect the LiPo battery
-  Connect the autopilot to the Mission Planner using a USB cable or telemetry
-  Open the Mission Planner's Initial Setup >> Optional Hardware >> Motor Test page

   .. image:: ../images/MissionPlanner_MotorTest.png
       :target: ../_images/MissionPlanner_MotorTest.png

-  Increase the "Throttle %" field and push each of the "Test motor" buttons to determine what percentage is required for each of the motors to spin.  If all ESCs are from the same manufacturer they will likely all have similar dead zones, but having one or two different by 1% ~ 2% is common.  Pick the highest percentage of all the motors - we will use this below.

Setting Spin-Armed and Min Throttle
===================================

By default, when the vehicle is armed but not flying, the motors will spin at a slightly slower than normal speed.  This speed can be configured using :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` paramter (For older versions check MOT_SPIN_ARMED).
Once the vehicle is flying, we want to ensure that we never output a value that causes the motors to stop spinning, this lower limit can be configured with the :ref:`THR_MIN<THR_MIN>` or :ref:`MOT_SPIN_MIN<MOT_SPIN_MIN>` parameter (depending upon the version).

If using Copter-3.3 (or earlier):

-  set the :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` parameter to **(the percentage discovered above + 2%) * 10**.  I.e. if you found the deadzone of the ESCs was 7%, set :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` to 90 (i.e. (7 + 2) * 10).
-  set the :ref:`THR_MIN<THR_MIN>` parameter to at least 30 higher than :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>`.  I.e. if :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` was 90, set :ref:`THR_MIN<THR_MIN>` to 120.

If using Copter-3.4 (or higher):

-  set the :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` parameter to **(the percentage discovered above + 2%) / 100**.  I.e. if you found the deadzone of the ESCs was 7%, set :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` to 0.09 (i.e (7 + 2) / 100).
-  set the :ref:`MOT_SPIN_MIN <MOT_SPIN_MIN>` parameter to at least 0.03 higher than :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>`.  I.e. if :ref:`MOT_SPIN_ARM <MOT_SPIN_ARM>` was 0.09, set :ref:`MOT_SPIN_MIN <MOT_SPIN_MIN>` to 0.12.

.. note::

   Setting :ref:`THR_MIN <THR_MIN>` or :ref:`MOT_SPIN_MIN <MOT_SPIN_MIN>` even higher than recommended above is fine especially because we want to account for voltage drop of the battery but setting it too high reduces the lower range of the motors which reduces control which could be important especially on powerful copters with a low hover throttle.

.. note::

   Copter 3.4 (and higher) also includes the :ref:`MOT_SPIN_MAX <MOT_SPIN_MAX>` parameter to account for the very top of the ESC/motor range which generally produces no additional thrust.  By default this value is 0.95 (i.e. top 5% of the range produces no additional thrust).
