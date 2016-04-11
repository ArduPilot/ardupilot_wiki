.. _quadplane-support:

=================
QuadPlane Support
=================

This article explains how to set up and use a combined fixed wing and
multicopter aircraft, also known as a "QuadPlane".

.. image:: ../images/quadplane_senior_telemaster.jpg
    :target: ../_images/quadplane_senior_telemaster.jpg

Overview
========

A QuadPlane is a combined fixed wing and MultiCopter aircraft. This sort
of aircraft brings the benefit of vertical takeoff and landing,
significantly greater speed and range of travel, and the ability to
hover and perform copter-like tasks at the destination (limited by
available power).

QuadPlane is built upon Plane, but adds control over between 4 and 8
horizontal rotors. Additional modes and commands allow a QuadPlane to
take off, land and fly like a copter, and to smoothly transition
between the Plane and Copter-like modes in both automatic and
autopilot-assisted modes. The additional rotors can also provide lift
and stability in normal Plane modes.

Support for QuadPlane simulation is available in SITL.

Getting the code
================

QuadPlane support is in APM:Plane releases from 3.5.0 onwards. The
normal instructions for installing the Plane firmware apply.

Flight modes
============

The QuadPlane code is based upon the Plane master firmware but with 3
extra modes:

-  mode 17: QSTABILIZE (like :ref:`Copter STABILIZE <copter:stabilize-mode>`)
-  mode 18: QHOVER (like :ref:`Copter ALT_HOLD <copter:altholdmode>`)
-  mode 19: QLOITER (like :ref:`Copter LOITER <copter:loiter-mode>`)
-  mode 20: QLAND (like :ref:`Copter LAND <copter:land-mode>`)

.. tip::

   You may probably need to set the ``FLTMODE*`` parameters for these
   extra modes as numeric values if your GCS doesn't understand these
   values yet.

If you are familiar with the equivalent Copter flight modes then you
should be comfortable flying a QuadPlane. The only real difference comes
during transition between fixed wing and QuadPlane flight, which is
described below.

.. note::

   The landing detection logic in QLOITER mode is not as
   sophisticated as the landing detection logic in Copter, so if you get
   GPS movement while on the ground in QLOITER mode then the aircraft may
   try to tip over as it tries to hold position while in contact with the
   ground. It is suggested that you switch to QHOVER or QSTABILIZE once
   landed as these are unaffected by GPS movement.

Flight modes to avoid
---------------------

The linked nature of of the vertical lift and fixed wing control in
quadplanes means the autopilot always needs to know what the pilot is
trying to do in terms of speed and attitude. For this reason you should
avoid the following flight modes in a quadplane:

-  ACRO
-  STABILIZE
-  TRAINING

these modes are problematic as the stick input from the pilot is not
sufficient to tell the autopilot what attitude the aircraft wants or
what climb rate is wanted, so the quadplane logic does not engage the
quad motors when in these modes. These modes also make log analysis
difficult. Please use FBWA mode instead of STABILIZE for manual flight.

In the future we may adds ways to use the quad motors in these modes,
but for now please avoid them.

The other mode where the quad motors are disabled is MANUAL mode. That
mode still can be useful for checking aircraft trim in fixed wing
flight, but make sure you only enter MANUAL mode once you have plenty of
airspeed.

Frame setup
===========

The code supports several frame arrangements of quadcopter,
hexacopter, octacopter and octaquad multicopter frames.

The motor order and output channel is the same as for copter (see :ref:`Copter motor layout <copter:connect-escs-and-motors>`)
except that the output channel numbers start at 5 instead of 1.

For example, with the default Quad-X frame the motors are on outputs
5 to 8. The arrangement is:

-  **Channel 5:** Front right motor
-  **Channel 6:** Rear left motor
-  **Channel 7:** Front left motor
-  **Channel 8:** Rear right motor

The normal plane outputs are assumed to be on 1 to 4 as usual. Only
outputs 5 to 8 run at high PWM rate (400Hz). You can also use channels 9
to 14 in any way you like, just as with the normal Plane code.

You can optionally move the quad motors to be on any other channel above
4, using the procedure outlined below.

To use a different frame type you can set Q_FRAME_CLASS and
Q_FRAME_TYPE. Q_FRAME_CLASS can be:

-  0 for quad
-  1 for hexa
-  2 for octa
-  3 for octaquad

Within each of these frame classes the Q_FRAME_TYPE chooses the motor
layout

-  0 for plus frame
-  1 for X frame
-  2 for V frame
-  3 for H frame

Using different channel mappings
--------------------------------

You can remap what output channels the quad motors are on by setting
values for RCn_FUNCTION. This follows the same approach as :ref:`other output functions <channel-output-functions>`.

Note that you do not need to set any of the RCn_FUNCTION values unless
you have a non-standard motor ordering. It is highly recommended that
you use the standard ordering.

The output function numbers are:

-  33: motor1
-  34: motor2
-  35: motor3
-  36: motor4
-  37: motor5
-  38: motor6
-  39: motor7
-  40: motor8

So to put your quad motors on outputs 9 to 12 (the auxillary channels on
a Pixhawk) you would use these settings in the advanced parameter list:

-  RC9_FUNCTION = 33
-  RC10_FUNCTION = 34
-  RC11_FUNCTION = 35
-  RC12_FUNCTION = 36

ESC calibration
===============

Most models of PWM based ESC need to be calibrated to ensure that all
the ESCs respond to the same input with the same speed. To calibrate
them they need to receive maximum PWM input when initially powered on,
then receive minimum PWM input when they have beeped to indicate that
the maximum has registered.

The quadplane code doesn't have a dedicated ESC calibration feature yet,
but you can use the following procedure to calibrate until that is
available:

#. remove your propellers for safety
#. power up just the flight board and not your motors. If you don't have
   the ability to isolate power to the ESCs when on battery power then
   power up your flight board on USB power
#. set both the parameters Q_M\_SPIN_ARMED and Q_THR_MID to 1000.
   This sets the PWM output when armed at zero throttle to full power
#. set the safety switch off to activate the outputs
#. arm your aircraft. The PWM output on all quad motors will now climb
   to maximum.
#. add power to your ESCs by connecting the battery
#. wait for the ESCs to beep to indicate they have registered the
   maximum PWM
#. disarm your aircraft. The ESCs should beep again indicating they have
   registered minimum PWM

Now set the Q_M\_SPIN_ARMED and Q_THR_MID parameters back to the
correct values. A value of 50 for Q_M\_SPIN_ARMED is a reasonable
starting point. For Q_THR_MID a value of between 500 and 600 is good
depending on the power of your motors

Transition
==========

You can transition between any modes, fixed wing or QuadPlane just by
changing mode. The transition rules are:

-  If you transition to :ref:`MANUAL <manual-mode>` then the quad motors
   will immediately stop.
-  If you transition to any other fixed wing mode then the quad will
   continue to supply lift and stability until you have reached the
   :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>` airspeed (or airspeed estimate if no airspeed sensor).
-  Once that airspeed is reached the quad motors will slowly drop in
   power over Q_TRANSITION_MS milliseconds (default is 5000, so 5
   seconds) and will switch off after that

If you transition from a fixed wing mode to a QuadPlane mode then the
fixed wing motor will immediately stop, but the control surfaces will
continue to provide stability while the plane slows down. This allows
for transitions to QuadPlane modes while flying at high speed.

.. note::

   If you transition to QLOITER while flying at high speed then the
   loiter code will try to bring the aircraft to a very rapid stop which
   will cause the plane to pitch up hard and then fly backwards to get back
   to the point where QLOITER was entered. Unless you are sure of the
   strength of your airframe it would be a good idea to transition to
   QHOVER first which will result in a much gentler transition, then move
   to QLOITER once the aircraft has slowed down.

Parameter setup
===============

All QuadPlane specific parameters start with a "Q\_" prefix. The
parameters are very similar to the equivalent Copter parameters so if
you are familiar with those you should find setting up a QuadPlane is
easy.

Key parameters are:

-  To enable QuadPlane functionality you need to set the Q_ENABLE
   parameter to 1 and then refresh the parameter list
-  The Q_THR_MIN_PWM and Q_THR_MAX_PWM parameters used to set the
   PWM range of the quad motors (this allows them to be different from
   the range for the forward motor). These need to be set to the range
   your ESCs expect.
-  You should set :ref:`SCHED_LOOP_RATE <SCHED_LOOP_RATE>`
   to 300 so the code runs the main loop at 300Hz, which is a good rate
   for both fixed wing and quadplane VTOL.
-  The most critical tuning parameters are Q_RT_RLL_P and
   Q_RT_PIT_P. These default to 0.15 (same as Copter) but you may
   find significantly higher values are needed for a QuadPlane.
-  The Q_M\_SPIN_ARMED parameter is important for getting the right
   level of motor output when armed in a quad mode
-  It is recommended that you set ARMING_RUDDER to 2 to allow for
   rudder disarm. Alternatively you could have :ref:`MANUAL <manual-mode>`
   as one of your available flight modes (as that will shut down the
   quad motors). Please be careful not to use hard left rudder and zero
   throttle while flying or you risk disarming your motors.
-  The Q_THR_MID parameter is important for smooth transitions. It
   defaults to 500 which means 50% throttle for hover. If your aircraft
   needs more or less than 50% throttle to hover then please adjust
   this. That will prevent a throttle surge during transition as the
   altitude controller learns the right throttle level

.. note::

   The QuadPlane code requires GPS lock for proper operation. This is
   inherited from the plane code, which disables intertial estimation of
   attitude and position if GPS lock is not available. Do not try to fly a
   QuadPlane indoors. It will not fly well

Assisted fixed-wing flight
==========================

The QuadPlane code can also be configured to provide assistance to the
fixed wing code in any flight mode except :ref:`MANUAL <manual-mode>`. To
enable quad assistance you should set Q_ASSIST_SPEED parameter to the
airspeed below which you want assistance.

When Q_ASSIST_SPEED is non-zero then the quad motors will assist with
both stability and lift whenever the airspeed drops below that
threshold. This can be used to allow flying at very low speeds in
:ref:`FBWA <fbwa-mode>` mode for example, or for assisted automatic fixed
wing takeoffs.

It is suggested that you do initial flights with ``Q_ASSIST_SPEED=0``
just to test the basic functionality and tune the airframe. Then try
with Q_ASSIST_SPEED above plane stall speed if you want that
functionality.

What assistance the quad motors provides depends on the fixed wing
flight mode. If you are flying in an autonomous or semi-autonomous mode
then the quad motors will try to assist with whatever climb rate and
turn rate the autonomous flight mode wants. In a manually navigated mode
the quad will try to provide assistance that fits with the pilot inputs.

The specific handling is:

-  In :ref:`AUTO <auto-mode>` mode the quad will provide lift to get to the
   altitude of the next waypoint, and will help turn the aircraft at the
   rate the navigation controller is demanding.
-  In fixed wing :ref:`LOITER <loiter-mode>`, :ref:`RTL <rtl-mode>` or GUIDED
   modes the quad motors will try to assist with whatever climb rate and
   turn rate the navigation controller is asking for.
-  In :ref:`CRUISE <cruise-mode>` or :ref:`FBWB <fbwb-mode>` mode the quad
   will provide lift according to the pilots demanded climb rate
   (controlled with pitch stick). The quad motors will try to turn at
   the pilot demanded turn rate (combining aileron and rudder input).
-  In :ref:`FBWA <fbwa-mode>` mode the quad will assume that pitch stick
   input is proportional to the climb rate the user wants. So if the
   user pulls back on the pitch stick the quad motors will try to climb,
   and if the user pushes forward on the pitch stick the quad motors
   will try to provide a stable descent.
-  In :ref:`AUTOTUNE <autotune-mode>` mode the quad will provide the same
   assistance as in :ref:`FBWA <fbwa-mode>`, but it is not a good idea to
   use :ref:`AUTOTUNE <autotune-mode>` mode with a high value of
   Q_ASSIST_SPEED as the quad assistance will interfere with the
   learning of the fixed wing gains.
-  In :ref:`MANUAL <manual-mode>`, :ref:`ACRO <acro-mode>` and
   :ref:`TRAINING <training-mode>` modes the quad motors will completely
   turn off. In those modes the aircraft will fly purely as a fixed
   wing.
-  In :ref:`STABILIZE <stabilize-mode>` mode the quad motors will try to
   provide lift if assistance is turned on.

Autonomous flight
=================

You can also ask the QuadPlane code to fly :ref:`AUTO <auto-mode>`
missions. To do that you plan an :ref:`AUTO <auto-mode>` mission as usual
and send a DO_VTOL_TRANSITION with parameter 1 equal to 3 to ask the
aircraft to switch to VTOL mode while flying the mission. When you do
that the fixed wing motor will stop and the aircraft will continue the
mission as a quadcopter. You can then send a DO_VTOL_TRANSITION with
parameter 1 equal to 4 to switch back to fixed wing flight.

The smooth transition rules apply to transitions in :ref:`AUTO <auto-mode>`
mode as they do for other modes, plus quad assistance applies in auto
fixed-wing mode if Q_ASSIST_SPEED is enabled.

In addition to DO_VTOL_TRANSITION the QuadPlane code supports two new
mission commands:

-  NAV_VTOL_TAKEOFF
-  NAV_VTOL_LAND

These mission commands can be used as part of a full auto mission to
give a vertical takeoff, followed by smooth transition to auto fixed
wing flight and then a vertical landing.

What will happen?
=================

Understanding hybrid aircraft can be difficult at first, so below are
some scenarios and how the ArduPilot code will handle them.

I am hovering in QHOVER and switch to FBWA mode
-----------------------------------------------

The aircraft will continue to hover, waiting for pilot input. If you
take your hands off the sticks at zero throttle the aircraft will
continue to hold the current height and hold itself level. It will drift
with the wind as it is not doing position hold.

If you advance the throttle stick then the forward motor will start and
the aircraft will start to move forward. The quad motors will continue
to provide both lift and stability while the aircraft is moving slowly.
You can control the attitude of the aircraft with roll and pitch stick
input. When you use the pitch stick (elevator) that will affect the
climb rate of the quad motors. If you pull back on the elevator the quad
motors will assist with the aircraft climb. If you push forward on the
pitch stick the power to the quad motors will decrease and the aircraft
will descend.

The roll and pitch input also controls the attitude of the aircraft, so
a right roll at low speed will cause the aircraft to move to the right.
It will also cause the aircraft to yaw to the right (as the QuadPlane
code interprets right aileron in fixed wing mode as a commanded turn).

Once the aircraft reaches an airspeed of :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`
(or Q_ASSIST_SPEED if that is set and is greater than :ref:`ARSPD_FBW_MIN <ARSPD_FBW_MIN>`)
the amount of assistance the quad motors provide will decrease over 5
seconds. After that time the aircraft will be flying purely as a fixed wing.

I am flying fast in FBWA mode and switch to QHOVER mode
-------------------------------------------------------

The quad motors will immediately engage and will start by holding the
aircraft at the current height. The climb/descent rate is now set by the
throttle stick, with a higher throttle stick meaning climb and a lower
throttle stick meaning descend. At mid-stick the aircraft will hold
altitude.

The forward motor will stop, but the aircraft will continue to move
forward due to its momentum. The drag of the air will slowly bring it to
a stop. The attitude of the aircraft can be controlled with roll and
pitch sticks (aileron and elevator). You can yaw the aircraft with
rudder.

I am flying fast in FBWA mode and switch to QLOITER mode
--------------------------------------------------------

The quad motors will immediately engage and the aircraft will pitch up
hard, as it tries to hold position at the position it was in when you
switched to QLOITER mode.

The aircraft will stop very quickly, and will back up slightly to the
position where QLOITER was entered. The movement of the aircraft can be
controlled with roll and pitch sticks (aileron and elevator). You can
yaw the aircraft with rudder.

The climb/descent rate is now set by the throttle stick, with a higher
throttle stick meaning climb and a lower throttle stick meaning descend.
At mid-stick the aircraft will hold altitude.

I switch to RTL mode while hovering
-----------------------------------

The aircraft will transition to fixed wing flight. The quad motors will
provide assistance with lift and attitude while the forward motor starts
to pull the aircraft forward.

The normal Plane RTL flight plan will then be run, which defaults to
circling at the RTL altitude above the arming position or nearest rally
point. If you have :ref:`RTL_AUTOLAND <RTL_AUTOLAND>`
setup then the aircraft will do a fixed wing landing.

When the aircraft is close to home you could switch it to QHOVER or
QLOITER to land vertically.

Typical flight
==============

A typical test flight would be:

-  takeoff in QLOITER or QHOVER
-  switch to :ref:`FBWA <fbwa-mode>` mode and advance throttle to start
   flying fixed wing
-  switch to QHOVER mode to go back to quad mode.

Simulation
==========

A simple QuadPlane model is available in SITL, allowing you to test the
features of the QuadPlane code without risking a real aircraft.

You can start it like this:

::

    sim_vehicle.sh -j4 -f quadplane --console --map

A parameter file to setup your QuadPlane is in **Tools/autotest**:

::

    param load ../Tools/autotest/quadplane.parm

To visualise the aircraft you can use FlightGear in view-only mode. The
simulation will output FlightGear compatible state on UDP port 5503.
Start FlightGear using the **fg_plane_view.sh** scripts provided in
the **Tools/autotest** directory.

Note that to get good scenery for FlightGear it is best to use a major
airport. I tend to test at San Francisco airport, like this:

::

    sim_vehicle.sh -L KSFO -f quadplane --console --map

Using the joystick module with a USB adapter for your transmitter gives
a convenient way to get used to the QuadPlane controls before flying.

If flying at KSFO there is a sample mission available with VTOL takeoff
and landing:

::

    wp load ../Tools/autotest/ArduPlane-Missions/KSFO-VTOL.txt

As usual you can edit the mission using "module load misseditor"

Building a QuadPlane
====================

Putting together a QuadPlane can be a daunting task. To help with ideas,
here are some links to some build logs that provide useful hints:

-  Porter OctaQuadPlane build:
   http://diydrones.com/profiles/blogs/building-flying-and-not-crashing-a-large-octaquadplane
-  Porter QuadPlane build:
   http://diydrones.com/profiles/blogs/building-flying-and-crashing-a-large-quadplane
-  QuadRanger build: http://px4.io/docs/quadranger-vtol/
