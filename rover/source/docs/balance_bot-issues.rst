.. _balance_bot-issues: 

========================
Commmon Issues and Fixes
========================

This page details some of the most common issues with Balance Bots and possible solutions.

1) Manual/Hold Mode issues:
===========================

The vehicle struggles to balance or wobbles too much
----------------------------------------------------

**1) Motor Backlash** : 
The balancing depends on very fine adjustments of the wheel. So any backlash between gears of the motor could lead to poor balancing performance. 

This is bad:

.. youtube:: 4Lkcze44W3E
    :width: 100%

The only fix is to change motors. Make sure the backlash is as less as possible.

**2) Weight Distribution** : 
If the center of gravity is near the wheels, the vehicle can topple quickly and if the motors are not fast enough to compensate, then the vehicle can become wobbly. The higher the center of gravity, the slower the vehicle falls. But more torque would be required from the motors to keep it balanced.

A good way to change the weight distribution is to move up/down the position of the batteries. How high depends on the torque and speed of the motors. In case of very fast motors with low torque, keep it closer to the wheels. If instead the motors are slow, but can provide high torque, then shift the center of mass higher by placing the batteries higher.


**3) Minimum Throttle** : 
If the minimum throttle sent to the motors from the autopilot is **too low** to turn on the motors, then the Balance Bot will move back and forth struggling to keep balance. Sometimes this issue can also cause it to drift off in one direction and topple. If the minimum throttle value is **too high** then the Balance Bot will be able maintain balance, but will be very wobbly. Refer to this guide to set the :ref:`minimum throttle <balance_bot-configure-throttle>` value correctly.

**4) PID tuning** : 
Improper PID tuning, especially high P or I gains can cause the vehicle to become wobbly. An insufficient D gain can also make the vehicle wobbly, but a very high D gain can cause very fast wobbling.


The vehicle drifts off in one direction
---------------------------------------

**1) Accelerometer Calibration** :
This can happen because the accelerometer calibration is invalid. Disturbing the autopilot, changing connections, crashes etc can disturb the acceleromter calibration. Do the accelerometer calibration again. Often, a one axis acccelerometer trim can fix this. 

**2) Pitch trim** :
The center of mass of the vehicle be slightly displaced from the zero pitch position. Hence the vehicle is not in equilibrium at 0 degrees pitch. This can be offset by :ref:`setting the pitch trim <balance_bot-tuning-pitch-trim>` parameter.

Configuration Issues
--------------------
Refer to the steps in the :ref:`configuration page<balance_bot-configure>` if you encounter any of the following issues:

- Vehicle does not respond to RC input
- Vehicle does not try to balance
- Motors are not moving

2) Acro Mode Issues
===================

The vehicle drives well, but randomly topples
---------------------------------------------
This happens when GPS lock occurs while the vehicle is indoors. The best solution is to disable GPS while the vehicle is used indoors. Set parameter:

- :ref:`GPS_TYPE<GPS_TYPE>` = 0 (Set this to 1 when outdoors)

The vehicle topples at higher speeds
------------------------------------
The motors may not have sufficient torque to balance the vehicle at these speeds. Lower the maximum speed of the vehicle by bringing down the :ref:`CRUISE_SPEED <CRUISE_SPEED>` parameter. Another option to consider is to reduce :ref:`BAL_PITCH_MAX<BAL_PITCH_MAX>` , which is the maximum pitch angle in degrees.

Vehicle tends to drift or yaw over time, without input
------------------------------------------------------
**1) Enable compass** (if not enabled already): Set parameters

- :ref:`COMPASS_USE <COMPASS_USE>` = 1
- :ref:`COMPASS_ENABLE <COMPASS_ENABLE>` = 1

**2) PID tuning** : Refer the :ref:`Acro tuning <balance_bot-tuning-acro>` page for more details

3) Auto Mode Issues
===================

Vehicle crashes after an Auto mission
-------------------------------------
This can happen if the vehicle switched to Hold after an Auto mission. To switch to Acro instead, set:

- :ref:`MIS_DONE_BEHAVE <MIS_DONE_BEHAVE>` =2
 
