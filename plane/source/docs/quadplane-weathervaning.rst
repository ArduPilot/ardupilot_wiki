.. _quadplane-weathervaning:

Weathervaning and Wind Hold
===========================

Flying a QuadPlane in significant levels of wind can present a
challenge. The issue is that the large wing surface offers a lot of
surface area for the wind to interact with. That can lead to a
reduction in attitude and position control and high motor and ESC
load.

To reduce the impact of wind when flying in VTOL modes the ArduPilot
QuadPlane code supports two features:

-  Active weathervaning
-  Position hold using forward motor

Together these two features can greatly reduce the impact of wind on
VTOL flight by keeping the aircraft pointed into the wind and reducing
the area of the wing exposed to the wind.

Active Weathervaning
--------------------

Active weathervaning acts to turn the nose of the aircraft into the
wind when flying in position controlled VTOL modes. You can enable
active weathervaning by setting the :ref:`Q_WVANE_GAIN <Q_WVANE_GAIN>`
parameter to a non-zero value. The default is not to use active
weathervaning.

The way it works is the autopilot looks at the roll attitude needed to
control the desired position. The basic algorithm is "turn into the
roll". If the aircraft needs to roll to the right in order to hold
position then it will turn in that direction on the assumption that
the right roll is needed in order to hold against the wind.

How quickly the aircraft turns is given by the Q_WVANE_GAIN
parameter. A good value to start with is 0.1. Higher values will make
the aircraft turn into the roll more quickly. If the value is too high
then you can get instability and oscillation in yaw.

To cope with a small amount of trim in the aircraft there is an
additional parameter :ref:`Q_WVANE_MINROLL <Q_WVANE_MINROLL>` which
controls the minimum roll level before weathervaning will be
used. This defaults to one degree. If you find your aircraft starts
yawing even in no wind then you may need to raise this value.

Active weathervaning is only active in VTOL modes, and VTOL sections
of AUTO modes (such as VTOL takeoff and VTOL landing). It is not
active in QSTABILIZE and QHOVER modes as those are not position
controlled modes. It is active is QLOITER, QLAND and QRTL modes.

Using the Forward Motor
-----------------------

In addition to active weathervaning, the QuadPlane code supports using
the forward motor to hold the pitch level in VTOL flight modes. To
enable use of the forward motor for position hold you need to set the
Q_VFWD_GAIN parameter to a non-zero value.

The way it works is to look at two factors:

-  the navigation attitude pitch of the aircraft
-  the difference between the desired forward velocity and the actual
   forward velocity

These are combined with the Q_VFWD_GAIN to ramp up and down the
throttle on the forward motor in order to minimize the attitude pitch
of the aircraft. That keeps the area of wing exposed to the wind
minimized which can reduce VTOL motor load.

A good value to start with for Q_VFWD_GAIN is 0.05. Higher values will
use the forward motor more aggressively. If the value is too high you
can get severe pitch oscillations.

Note that you can also use reverse thrust on the forward motor. If
your :ref:`THR_MIN <THR_MIN>` parameter is less than zero then reverse
thrust is available and the motor will use reverse thrust to slow down
or move backwards as needed. See the :ref:`reverse thrust
<reverse-thrust>` section in the :ref:`automatic landing
<automatic-landing>` documentation for more details.

As with active weathervaning, using the forward motor is only enabled
in position controlled VTOL modes. This means it is not enabled in
QSTABILIZE or QHOVER flight modes. It is available in QLOITER, QRTL,
QLAND and in AUTO mode when executing VTOL flight commands.

