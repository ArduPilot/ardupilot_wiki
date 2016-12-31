.. _channel-output-functions:

========================
Channel Output Functions
========================

The first 4 output channels in Plane have specific meanings - typically
Aileron, Elevator, Throttle and Rudder. Beyond that you may map the
remaining output channels to any of a large number of output functions.
On a board like the Pixhawk which has 14 output channels that means you
have 10 channels that you can freely assign to any function.

This page describes how to configure these output channels and what each
of the available functions is.

The RCn_FUNCTION parameters
----------------------------

In the advanced parameter view of your GCS you will find that each RC
output channel beyond channel 4 has a RCn_FUNCTION parameter. For
example, RC5_FUNCTION controls the output function of channel 5,
RC6_FUNCTION controls the output function of channel 6 and so on.

The values you can set these parameters to are shared with copters and
rovers, and not all of them have been implemented on fixed wing
aircraft. The ones that are implemented on fixed wing are listed below:

-  Disabled=0
-  RCPassThru=1
-  Flap=2
-  Flap_auto=3
-  Aileron=4
-  mount_pan=6
-  mount_tilt=7
-  mount_roll=8
-  camera_trigger=10
-  mount2_pan=12
-  mount2_tilt=13
-  mount2_roll=14
-  DifferentialSpoiler1=16
-  DifferentialSpoiler2=17
-  AileronWithInput=18
-  Elevator=19
-  ElevatorWithInput=20
-  Rudder=21
-  Flaperon1=24
-  Flaperon2=25
-  GroundSteering=26

The default value for all channels is 0, meaning disabled. That means
the channel will output the trim value for that channel (for example, if
RC5_FUNCTION is 0 then channel 5 will output RC5_TRIM) unless it is
overridden by a mission command.

All of these functions may be used on multiple channels. So if you want
3 more elevator channels for some reason you can set RCn_FUNCTION to 19
on 3 of your output channels. The normal elevator (channel 2) will still
work, but you will also have 3 extra elevons, each of which you can trim
and set the range of separately.

Disabled
--------

For normal operation, the Disabled output function sets the output value
of the channel to the trim value. The exception to this is when a
MAVLink override of the channel or a mission servo set is used. So in
some ways "disabled" could be called "mission-controlled".

When you fly an auto mission you can ask for a servo to be set to a
value as part of that mission. In that case you should set the
RCn_FUNCTION for that channel to Disabled, so that the value doesn't
get changed by another output function immediately after the mission
sets the value.

RCPassThru
----------

Setting a channel to RCPassThru means it will output the value that is
coming into the board from the corresponding input channel. For example,
if RC5_FUNCTION is 1 (meaning RCPassThru) then channel 5 output will
always be equal to channel 5 input.

.. _channel-output-functions_flap:

Flap
----

When a channel is set as a flap it's value comes from the flap input
channel (controlled by the FLAP_IN_CHANNEL parameter). The reason you
may want to use this instead of a RCPassThru is that you can setup
multiple flap channels with different trims and ranges, and you may want
to take advantage of the FLAP_SLEWRATE to limit the speed of flap
movement.

.. _channel-output-functions_flap_auto:

Flap_auto
----------

The flap auto output function behaves like the Flap output, except it
can also accept automatic flap output from the TKOFF_FLAP_PCNT and
LAND_FLAP_PERCNT parameters, as well as the FLAP_1\_SPEED,
FLAP_1\_PERCNT, FLAP_2\_SPEED and FLAP_2\_PERCNT parameters.

If you have both a FLAP_IN_CHANNEL set and a Flap_auto output
function set then the amount of flap applied is the higher of the two.

Aileron
-------

The aileron output function adds additional aileron outputs, with
separate per-channel trim and range. This is useful when you want to
trim each aileron separately, or if your main aileron is setup as an
elevon mixer (using the ELEVON_OUTPUT option), and you also want some
normal ailerons.

AileronWithInput
----------------

The AileronWithInput function is used where you have setup your
transmitter to input the aileron signal you want on this channel on the
corresponding input channel for this output. For example, if you set
RC6_FUNCTION=18 and have setup your transmitter to send the right
aileron signal for manual mode to channel 6 then you can use
AileronWithInput. The main difference is in manual mode. In manual with
AileronWithInput the output comes directly from the corresponding input
channel. With the Aileron output function the output in manual mode is
based on the main aileron input channel (usually channel 1) but trimmed
and scaled according to the RC6 trim values.

Mount_pan, Mount_tilt and Mount_roll
---------------------------------------

These control the output channels for controlling a servo gimbal. Please
see the :ref:`camera gimbal configuration documentation <common-camera-gimbal>` for details.

The Mount2_pan, Mount2_tilt and Mount2_roll options are the same, but
control a second camera gimbal

Camera_trigger
---------------

The Camera_trigger output function is used to trigger a camera with a
servo. See the :ref:`camera gimbal documentation <common-camera-gimbal>` for details.

Elevator
--------

The elevator output function adds additional elevator outputs, with
separate per-channel trim and range. This is useful when you want to
trim each elevator separately, or if your main elevator is setup as an
elevon mixer (using the ELEVON_OUTPUT option), and you also want some
normal elevator.

ElevatorWithInput
-----------------

The ElevatorWithInput function is used where you have setup your
transmitter to input the elevator signal you want on this channel on the
corresponding input channel for this output. For example, if you set
RC6_FUNCTION=20 and have setup your transmitter to send the right
elevator signal for manual mode to channel 6 then you can use
ElevatorWithInput. The main difference is in manual mode. In manual with
ElevatorWithInput the output comes directly from the corresponding input
channel. With the Elevator output function the output in manual mode is
based on the main elevator input channel (usually channel 2) but trimmed
and scaled according to the RC6 trim values.

Rudder
------

The rudder output function adds additional rudder outputs, with separate
per-channel trim and range. Separate rudder channels is particularly
useful for nose wheel steering where the nose wheel may need to be
reversed as compared to the normal rudder channel or for multi-wheel
planes.

GroundSteering
--------------

The GroundSteering output function acts much like the rudder output
function except that it only acts when the aircraft is below
GROUND_STEER_ALT altitude. At altitudes above GROUND_STEER_ALT the
output will be the trim value for the channel.

.. _channel-output-functions_flaperon1_and_flaperon2:

Flaperon1 and Flaperon2
-----------------------

Using the flaperon1 and flaperon2 output functions you can setup
flaperons, which are ailerons that double as flaps. They are very useful
for aircraft which have ailerons but no flaps.

To use the flaperon output functions you need to also set the
FLAPERON_OUTPUT option to the right value (from 1 to 4) for your servo
setup. See the :ref:`Elevon setup page <reversing-servos-and-setting-normalelevon-mode>`
for a more detailed description of how this parameter works (that page
is for elevons, but flaperons work in the same manner).

Note that flaperons act like Flap_auto described above for the flap
component of the output.
