.. _traditional-helicopter-rsc-setup:

=========================
Rotor Speed Control Setup
=========================

The Rotor Speed Control (RSC) uses the heliRSC output (``SERVOx_FUNCTION`` = 31) to the engine throttle or speed control/governor to control rotor speed. This can be done in a few different ways:

-  Through an open loop control via a throttle curve driven by the RC channel used for collective input 
-  Using a set point signal to an external governor, or electronic speed controller
-  Using a closed loop governor in the ArduPilot firmware.

.. note:: When Motor Interlock is disengaged,the state is sometimes called Throttle Hold and throttle is driven to :ref:`H_RSC_IDLE <H_RSC_IDLE>`

The RSC also controls the heliRSC output for ground idle, rotor run up and shut down. This wiki covers the set up of each of the parameters in the RSC.

RSC mode
========

First, set the RSC Mode parameter (:ref:`H_RSC_MODE <H_RSC_MODE>`). The RSC modes are listed below with a short description. 

#. RC Passthrough - this mode passes through the RC channel input on which the Motor Interlock (``RCx_OPTION`` =32) is assigned. The channel must be over 1200us in order for the heliRSC output to follow the RC input. Otherwise, heliRSC will be :ref:`H_RSC_IDLE <H_RSC_IDLE>`.
#. RSC setpoint - this mode is used for helicopters utilizing either an electronic speed controller or an external governor for internal combustion engines. The PWM passed to the HeliRSC output is determined from the External Motor Governor Setpoint (:ref:`H_RSC_SETPOINT <H_RSC_SETPOINT>`) parameter. The output PWM is calculated by the following equation: PWM output = ``RSC_SETPOINT`` * 0.01 * (``SERVOx_MAX`` - ``SERVOx_MIN``) + ``SERVOx_MIN`` where SERVOx is the output assigned to Throttle
#. Throttle curve - This mode is an open loop control of the HeliRSC servo output. Users will need to fine-tune the throttle curve to maintain the desired rotor speed throughout the flight envelope. The throttle curve is a five point spline curve fit set by the ``H_RSC_THRCRV_x`` parameters. It is used to determine the HeliRSC servo output based on the collective (throttle stick) on the RC transmitter.
#. Governor - The governor is designed to maintain a user specified rotor speed using the throttle curve as the feedforward control. This feature requires a rotor speed sensor. The ``H_RSC_GOV_x`` 

.. warning::
    Setting the RSC mode to RC Passthrough requires configuring the RC receiver to hold last value for the Motor Interlock channel (default is channel 8). If the receiver loses connection to the transmitter and receiver is not configured correctly, the motor will shutdown and the helicopter will crash!


Rotor Speed Ramp and Idle Settings
==================================

The rotor speed control features an idle setting and start up and shut down logic for throttle control. The Throttle Output at Idle parameter (:ref:`H_RSC_IDLE <H_RSC_IDLE>`) determines the output to heliRSC servo output after the aircraft is armed, but before the motor interlock is enabled. 

When the motor interlock is enabled the rotor speed control will ramp  from the idle to flight setting based on the Throttle Ramp Time parameter (:ref:`H_RSC_RAMP_TIME <H_RSC_RAMP_TIME>`). The RSC will prevent take off in non-manual throttle modes and auto mode until the RSC Run Up Timer has completed. The run up time is specified by the Rotor Runup Time parameter (:ref:`H_RSC_RUNUP_TIME <H_RSC_RUNUP_TIME>`).  This parameter has to be equal to or greater than the Throttle Ramp Time parameter (:ref:`H_RSC_RAMP_TIME <H_RSC_RAMP_TIME>`). 

When the motor interlock is disabled, the rotor speed control will count down the same amount of time as specified by the RSC Run Up Timer. The RSC will declare rotor speed below critical based on the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`). It is best to set the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`) for a percentage of the runup timer that equates to about three seconds. For example if you had a 10 second runup timer, setting the Critical Rotor Speed parameter (:ref:`H_RSC_CRITICAL <H_RSC_CRITICAL>`) to 70% will cause the RSC to declare rotor speed below critical three seconds from when Motor interlock is disabled.

Governor Setup
==============
The internal rotor speed governor is a recent addition in Copter 4.0. `This discuss post <https://discuss.ardupilot.org/t/helicopter-rotor-speed-governor/35932>`__  describes the set up for this new feature.

