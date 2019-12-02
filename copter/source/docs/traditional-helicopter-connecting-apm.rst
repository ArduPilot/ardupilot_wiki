.. _traditional-helicopter-connecting-apm:

==============================================================
Traditional Helicopter — Connecting and Calibrating the System
==============================================================

Autopilot Info
==============

.. image:: ../images/pixhackv5.jpg
    :target: ../_images/pixhackv5.jpg

A autopilot with internally damped IMU's is highly recommended for 
helicopters. Experience has shown the tuning, handling and stability 
performance of your helicopter will be greatly improved over the first 
generation Pixhawk.

Before you begin connecting the system it is recommended to review the docs for
the autopilot you select.

Overview of servo, and RX connection
====================================

The RC input for many ardupilot compatible autopilots is either PPM SUM
(8 channels) or S.Bus (up to 18 channels).  Some controllers also accept Spektrum
satellite receivers.  If you have a receiver that outputs only PWM, you will need
a PPM encoder to connect to your autopilot.

The default RC input and SERVO output mapping is as follows:

+--------------+-------------+
| RC Channel   | SERVO output|
+--------------+-------------+
| 1            | Aileron     |
+--------------+-------------+
| 2            | Elevator    |
+--------------+-------------+
| 3            | Collective  |
+--------------+-------------+
| 4            | Rudder      |
+--------------+-------------+
| 5            | Flight Modes|
+--------------+-------------+
| 6            | Tuning      |
+--------------+-------------+
| 7            | Aux         |
+--------------+-------------+
| 8            | Throttle    |
+--------------+-------------+

The output pins on most controllers for SERVO's 1 thru 8 are labled Main Out:

.. image:: ../images/PH21_2.jpg
    :target: ../_images/PH21_2.jpg

Motor 1, which is normally the left front servo on your helicopter goes to pin 1.
Motor 2, which is normally the right front servo goes to pin 2.  Motor 3, which 
is normally the rear (elevator) servo goes to pin 3. Tail servo goes to pin 4.

If you are using a DDVP (Direct Drive Variable Pitch) tail rotor, the tail motor
ESC connects to pin 7. The throttle servo or ESC for the main rotor motor
connects to pin 8.

Check the docs for your selected autopilot but most require a separate 
power supply to the servo rail to power your servos at their appropriate rated 
voltage. 

Connect telemetry radios, GPS/compass module, power to autopilot itself,
and any other peripherals as per the instructions in the owners manual for the unit.

RC Calibration
--------------

.. warning::

   Before powering the autopilot and servo rail for the first time, 
   disconnect the rudder linkage from the tail servo or bellcrank on the tail 
   gearbox. If you have a piston engine helicopter, also disconnect the throttle
   servo linkage. 

The RC MUST be calibrated before proceeding once the autopilot is powered up.
RC calibration is identical to all other vehicles. With helicopters using the
ArduPilot system there can be no mixes in the RC radio. All the outputs must be
"pure", i.e. use either airplane mode in your radio, or helicopter mode with H1
or "straight" swash.
:ref:`See this topic <common-radio-control-calibration>`.

Compass Calibration
-------------------

It is recommended to calibrate the compasses at this time as well. This is the
same as all other vehicles.
:ref:`See this topic <common-compass-calibration-in-mission-planner>`.

Accelerometer Calibration
-------------------------
If the accelerometers were not calibrated on the bench prior to installation it
must be calibrated before proceeding.
:ref:`See this topic <common-accelerometer-calibration>`.
