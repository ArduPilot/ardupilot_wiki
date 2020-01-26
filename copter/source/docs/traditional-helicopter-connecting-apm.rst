.. _traditional-helicopter-connecting-apm:

==============================================================
Traditional Helicopter — Connecting and Calibrating the System
==============================================================

Autopilot Info
==============

.. image:: ../images/pixhackv5.jpg
    :target: ../_images/pixhackv5.jpg

A autopilot with internally damped IMU's is highly recommended for helicopters. Experience has shown the tuning, handling and stability performance of your helicopter will be greatly improved over the first generation Pixhawk.

Before you begin connecting the system it is recommended to review the docs for the autopilot you select.

Overview of servo, and RX connection
====================================

The RC input for many ardupilot compatible autopilots is either PPM SUM (8 channels) or S.Bus (up to 18 channels).  Some controllers also accept Spektrum satellite receivers.  For receivers that only output PWM, a PPM encoder is required to connect to the autopilot, however this is not recommended for applications requiring minimum possible latency.  For all helicopter frames, an RC radio that has at least 6 channels is required.  For flight controllers, traditional helicopter and quadheli frames require one that has at least 5 output channels. The dual helicopter frame requires the flight controller have at least 7 output channels. 

The default receiver channel to ArduCopter RC input function mapping is as follows:

+--------------+-----------------+
| RC Receiver  | Ardupilot RC    |
| Channel      | Input Function  |
+--------------+-----------------+
| 1 (Aileron)  | Roll (note)     |
+--------------+-----------------+
| 2 (Elevator) | Pitch (note)    |
+--------------+-----------------+
| 3 (Throttle) | Collective(note)|
+--------------+-----------------+
| 4 (Rudder)   | Yaw (note)      |
+--------------+-----------------+
| 5 (Gear)     | Flight Mode     |
+--------------+-----------------+
| 6 (Aux 1)    | Tuning          |
+--------------+-----------------+
| 7 (Aux 2)    | Aux             |
+--------------+-----------------+
| 8 (Aux 3)    | Motor Interlock |
|              | (throttle)      |
+--------------+-----------------+

Note: Functions are already mapped by RCMAP parameters. 

The output on most controllers for SERVO's 1 thru 8 are labled Main Out:

.. image:: ../images/PH21_2.jpg
    :target: ../_images/PH21_2.jpg

The :ref:`autopilot output functions wiki <common-rcoutput-mapping>` shows the complete list of servo output functions.  The default swashplate is H3-120 where Motor 1 (servo function 33) left front servo, goes to output 1; Motor 2 (output function 34), right front servo goes to output 2; and Motor 3 (output function 35), rear (elevator) servo goes to output 3. See the :ref:`swashplate setup wiki <traditional-helicopter-swashplate-setup>` for more details.

Tail servo is designated as Motor 4 (servo function 36) and is defaulted to output 4. Direct Drive Fixed Pitch (DDFP) tail rotors will also be connected to Motor 4 and the tail type parameter (:ref:`H_TAIL_TYPE <H_TAIL_TYPE>`) set to DDFP.  Direct Drive Variable Pitch (DDVP) tail rotors will use Motor 4 (defaulted to output 4) to control tail rotor pitch and the tail ESC (servo function 32) connection is defaulted to output 7. This is automatically configured as tail RSC for the servo 7 function when the tail type parameter (:ref:`H_TAIL_TYPE <H_TAIL_TYPE>`) is set to DDVP. 
 
The throttle servo or ESC for the main rotor motor is defaulted to output 8.  This is automatically configured as Heli RSC (servo function 31) for the servo 8 function. See the :ref:`rotor speed control setup wiki <traditional-helicopter-rsc-setup>` for more details on RSC setup.  All traditional helicopter frames are required to use Motor Interlock.  This feature adds an extra layer of safety when working with helicopters.  Motor interlock enables the motor to drive the rotor/tailrotor.  This is similar to throttle hold in RC helicopters.  Motor interlock enabled (throttle hold off) means the motor is allowed to drive the rotor/tailrotor and the rotor speed control handles the rotor runup/shutdown.  Motor interlock disabled (throttle hold on) means the motor is not allowed to drive the rotor/tailrotor.  In order to arm the helicopter, the motor interlock must be disabled (throttle hold on). In ArduCopter 3.6 and earlier, the motor interlock and RC passthrough mode is tied to RC channel 8 only.  The RC transmitter channel 8 must have the PWM within 10 pwm of the RC8_MIN for motor interlock disabled (throttle hold on).  All other PWM values will set motor interlock enabled (throttle hold off).  In ArduCopter 4.0, the RCn_Option parameter can be set to motor interlock for a user selectable channel.  The transmitter channel on which the motor interlock is set requires the PWM to be low (<1200 PWM) for motor interlock disabled (throttle hold on) and above 1200 PWM for motor interlock enabled (throttle hold off).  

Check the docs for your selected autopilot but most require a separate power supply to the servo rail to power your servos at their appropriate rated voltage. 

Connect telemetry radios, GPS/compass module, power to autopilot itself, and any other peripherals as per the instructions in the owners manual for the unit.

RC Calibration
--------------

.. warning::

   Before powering the autopilot and servo rail for the first time, 
   disconnect the rudder linkage from the tail servo or bellcrank on the tail 
   gearbox. If you have a piston engine helicopter, also disconnect the throttle
   servo linkage. 

The RC MUST be calibrated before proceeding once the autopilot is powered up. RC calibration is identical to all other vehicles. With helicopters using the ArduPilot system there can be no mixes in the RC radio. All the outputs must be
"pure", i.e. use either airplane mode in your radio, or helicopter mode with H1 or "straight" swash.
:ref:`See this topic <common-radio-control-calibration>`.

Compass Calibration
-------------------

It is recommended to calibrate the compasses at this time as well. This is the same as all other vehicles.
:ref:`See this topic <common-compass-calibration-in-mission-planner>`.

Accelerometer Calibration
-------------------------
If the accelerometers were not calibrated on the bench prior to installation it must be calibrated before proceeding.(It is usually easier to calibrate on the bench and then re-calibrate only the level position, if required, once installed.)
:ref:`See this topic <common-accelerometer-calibration>`.
