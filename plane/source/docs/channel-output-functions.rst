.. _channel-output-functions:

=================================
Archived:Channel Output Functions
=================================

This page superseded by :ref:`common-rcoutput-mapping`

All servo outputs may be mapped to any function supported by
ArduPilot. The default settings are for the first 4 channels to be
Aileron, Elevator, Throttle and Rudder (commonly known as AETR), but
you can re-assign them as needed.

This page describes how to configure these output channels and what each
of the available functions is.

The SERVOn_FUNCTION parameters
------------------------------

In the advanced parameter view of your GCS you will find that each
SERVO output channel has a SERVOn_FUNCTION parameter. For example,
:ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` controls the output function of channel 5,
:ref:`SERVO6_FUNCTION<SERVO6_FUNCTION>` controls the output function of channel 6 and so on.

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
-  DifferentialSpoilerLeft1=16
-  DifferentialSpoilerRight1=17
-  DifferentialSpoilerLeft2=86
-  DifferentialSpoilerRight2=87
-  Elevator=19
-  Rudder=21
-  FlaperonLeft=24
-  FlaperonRight=25
-  GroundSteering=26
-  ParachuteRelease=27
-  QuadPlaneMotor1=33
-  QuadPlaneMotor2=34
-  QuadPlaneMotor3=35
-  QuadPlaneMotor4=36
-  QuadPlaneMotor5=37
-  QuadPlaneMotor6=38
-  QuadPlaneMotor7=39
-  QuadPlaneMotor8=40
-  MotorTilt=41
-  RCPassThru1=51
-  RCPassThru2=52
-  RCPassThru3=53
-  RCPassThru4=54
-  RCPassThru5=55
-  RCPassThru6=56
-  RCPassThru7=57
-  RCPassThru8=58
-  RCPassThru9=59
-  RCPassThru10=60
-  RCPassThru11=61
-  RCPassThru12=62
-  RCPassThru13=63
-  RCPassThru14=64
-  RCPassThru15=65
-  RCPassThru16=66
-  Ignition=67
-  Starter=69
-  Throttle=70
-  ThrottleLeft=73
-  ThrottleRight=74
-  TiltMotorLeft=75
-  TiltMotorRight=76
-  ElevonLeft=77
-  ElevonRight=78
-  VTailLeft=79
-  VTailRight=80

The default values for the first 4 channels are Aileron, Elevator,
Throttle and Rudder. The default for all other channels s is 0,
meaning disabled. A disabled channel will output the trim value for
that channel (for example, if :ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` is 0 then channel 5 will
output SERVO5_TRIM) unless it is overridden by a mission command.

All of these functions may be used on multiple channels. So if you
want 3 elevator channels for some reason you can set SERVOn_FUNCTION
to 19 on 3 of your output channels.

Disabled
--------

For normal operation, the Disabled output function sets the output value
of the channel to the trim value. The exception to this is when a
MAVLink override of the channel or a mission servo set is used. So in
some ways "disabled" could be called "mission-controlled".

When you fly an auto mission you can ask for a servo to be set to a
value as part of that mission. In that case you should set the
SERVOn_FUNCTION for that channel to Disabled, so that the value doesn't
get changed by another output function immediately after the mission
sets the value.

RCPassThru
----------

Setting a channel to RCPassThru means it will output the value that is
coming into the board from the corresponding input channel. For example,
if :ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` is 1 (meaning RCPassThru) then channel 5 output will
always be equal to channel 5 input.

You can also map individual channels to any output channel by using the specific channel mapping functions. These are numbered starting at value 51, for RCInputChannel1. So you can for example set SERVO11_FUNCTION=53 which will map RC input channel 3 to output channel 11.

.. _channel-output-functions_flap:

Flap
----

When a channel is set as a flap it's value comes from the flap input
channel (controlled by the ``RCn_OPTION`` = 208 parameter). The reason you
may want to use this instead of a RCPassThru is that you can setup
multiple flap channels with different trims and ranges, and you may want
to take advantage of the :ref:`FLAP_SLEWRATE<FLAP_SLEWRATE>` to limit the speed of flap
movement.

.. _channel-output-functions_flap_auto:

Flap_auto
---------

The flap auto output function behaves like the Flap output, except it
can also accept automatic flap output from the :ref:`TKOFF_FLAP_PCNT<TKOFF_FLAP_PCNT>` and
:ref:`LAND_FLAP_PERCNT<LAND_FLAP_PERCNT>` parameters, as well as the FLAP_1\_SPEED,
FLAP_1\_PERCNT, FLAP_2\_SPEED and FLAP_2\_PERCNT parameters.

If you have both a FLAP RC input channel set and a Flap_auto output
function set then the amount of flap applied is the higher of the two.

Aileron
-------

The aileron output function adds additional aileron outputs, with
separate per-channel trim and range. This is useful when you want to
trim each aileron separately, or if your main aileron is setup as an
elevon mixer (using the ``SERVOn_FUNCTION`` ), and you also want some
normal ailerons.

Mount_pan, Mount_tilt and Mount_roll
------------------------------------

These control the output channels for controlling a servo gimbal. Please
see the :ref:`camera gimbal configuration documentation <common-camera-gimbal>` for details.

The Mount2_pan, Mount2_tilt and Mount2_roll options are the same, but
control a second camera gimbal

Camera_trigger
--------------

The Camera_trigger output function is used to trigger a camera with a
servo. See the :ref:`camera gimbal documentation <common-camera-gimbal>` for details.

Elevator
--------

The elevator output function adds additional elevator outputs, with
separate per-channel trim and range. This is useful when you want to
trim each elevator separately, or if your main elevator is setup as an
elevon mixer (using the ``SERVOn_FUNCTION`` ), and you also want some
normal elevator.

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
:ref:`GROUND_STEER_ALT<GROUND_STEER_ALT>` altitude. At altitudes above :ref:`GROUND_STEER_ALT<GROUND_STEER_ALT>` the
output will be the trim value for the channel.

.. _channel-output-functions_flaperons:

Flaperons
-----------------------

Using SERVOn_FUNCTION 24 and 25 (Flaperon Left / FlaperonRight) you can setup
flaperons, which are ailerons that double as flaps. They are very useful
for aircraft which have ailerons but no flaps.

See the :ref:`flaperon guide <flaperons-on-plane>` for more details.

Note that flaperons act like Flap_auto described above for the flap
component of the output.
