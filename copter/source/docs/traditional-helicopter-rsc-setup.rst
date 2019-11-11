.. _traditional-helicopter-rsc-setup:

=========================
Rotor Speed Control Setup
=========================

The Rotor Speed Control (RSC) uses the throttle output to the engine to control rotor speed. This can be done in a few different ways either through an open loop control called a throttle curve, a set point signal to an external governor or electronic speed controller, or a closed loop internal governor. The RSC also controls the throttle for ground idle, rotor run up and shut down. This wiki covers the set up of each of the parameters in the RSC.

RSC mode
========

First, set the RSC Mode parameter (:ref:`H_RSC_MODE <H_RSC_MODE>`). The RSC modes are listed below with a short description. 

* RC Passthrough - this mode passes through the RC channel input on which the motor interlock is assigned. 

.. warning::
    Setting the RSC mode to RC passthrough requires configuring the RC receiver to hold last value for the Motor Interlock channel (default is channel 8). If the receiver loses connection to the transmitter and receiver is not configured correctly, the motor will shutdown and the helicopter will crash!

* RSC setpoint - this mode is used for helicopters utilizing either an electronic speed controller or an external governor for internal combustion engines. The PWM passed to the Heli RSC channel is determined from the External Motor Governor Setpoint (:ref:`H_RSC_SETPOINT <H_RSC_SETPOINT>`) parameter. The output PWM is calculated by the following equation:
PWM output = RSC_SETPOINT * 0.01 * (SERVO_MAX - SERVO_MIN) + SERVO_MIN

* Throttle curve - This mode is an open loop control of the heliRSC servo. Users will need to fine-tune the throttle curve to maintain the desired rotor speed throughout the flight envelope. The throttle curve is a five point spline curve fit. It is used to determine the HeliRSC servo output based on the collective (throttle stick) on the RC transmitter. 

* Governor - The governor is designed to maintain a user specified rotor speed using the throttle curve as the feedforward control. This feature requires a rotor speed sensor.

Rotor Speed Ramp and Idle Settings
==================================

The rotor speed control features an idle setting and start up and shut down logic for throttle control. The Throttle Output at Idle parameter (:ref:`H_RSC_IDLE <H_RSC_IDLE>`) determines the output to HeliRSC servo after the aircraft is armed but before The motor interlock is enabled. When the motor interlock is enabled the rotor speed control will ramp throttle from the idle to flight setting based on the Throttle Ramp Time parameter (:ref:`H_RSC_RAMP_TIME <H_RSC_RAMP_TIME>`). The RSC will prevent take off in non-manual throttle modes and auto mode until the RSC run up timer has completed. The run up time is specified by the Rotor Runup Time parameter (:ref:`H_RSC_RUNUP_TIME <H_RSC_RUNUP_TIME>`).  This parameter has to be equal to or greater than the Throttle Ramp Time parameter (:ref:`H_RSC_RAMP_TIME <H_RSC_RAMP_TIME>`).  When the motor interlock is disabled, the rotor speed control will count down the same amount of time as specified by the RSC run up timer. The RSC will declare rotor speed below critical based on the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`). It is best to set the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`) for a percentage of the runup timer that equates to about three seconds. For example if you had a 10 second runup timer, setting the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`) to 70% will cause the RSC to declare rotor speed below critical three seconds from when Motor interlock is disabled.

Governor Setup
==============
The internal rotor speed governor is a recent addition in Arducopter 4.0. This discuss post describes the set up for this new feature.
https://discuss.ardupilot.org/t/helicopter-rotor-speed-governor/35932
