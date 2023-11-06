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

-  Active yaw axis weathervaning
-  Position hold assistance using forward or tilt rotor motors instead of only using VTOL stance tilting

Together these two features can greatly reduce the impact of wind on
VTOL flight by keeping the aircraft pointed into the wind and reducing
the area of the wing exposed to the wind.

Active Weathervaning
--------------------

Active weathervaning acts to turn the aircraft into the
wind when flying in position-controlled VTOL modes. You can enable
active weathervaning by setting the :ref:`Q_WVANE_ENABLE <Q_WVANE_ENABLE>`
parameter to a non-zero value. The default is not to use active
weathervaning.

+---------------------------------------------+--------------------------+
+ :ref:`Q_WVANE_ENABLE<Q_WVANE_ENABLE>` Value | Weathervane Direction    +
+=============================================+==========================+
+         0                                   |  Disabled                +
+---------------------------------------------+--------------------------+
+         1                                   |  Nose Into Wind(default) +
+---------------------------------------------+--------------------------+
+         2                                   |  Nose OR Tail Into Wind  +
+---------------------------------------------+--------------------------+
+         3                                   |  Side Into Wind          +
+---------------------------------------------+--------------------------+
+         4                                   |  Tail Into Wind          +
+---------------------------------------------+--------------------------+

The way it works is the autopilot looks at the roll and/or pitch attitude needed to
control the desired position. The basic algorithm is "yaw into the
roll/pitch". If the aircraft needs hold roll to the right in order to hold
position then it will turn in that direction on the assumption that
the right roll is needed in order to hold against the wind (assuming nose into the wind). Similarly, for Side Into the Wind, it will use the pitch angle and yaw appropriately, trying to zero the pitch required to hold position.

.. note:: by default, weathervaning does not use pitch, only roll. This is to prevent unwanted continuous yawing if the hover attitude has not been trimmed with :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` to hover, in place, with no wind. If this **has** been done, then pitch driven weathervaning can be enabled by setting :ref:`Q_WVANE_OPTIONS<Q_WVANE_OPTIONS>` bit 0 to "1". This will speed weathervaning if positioned with the wind blowing from behind (ie mostly being held in position with pitch, not roll). This does not affect Side Into Wind, it always uses pitch, and :ref:`Q_TRIM_PITCH<Q_TRIM_PITCH>` must be properly set for good operation.

How quickly the aircraft yaws is determined by the :ref:`Q_WVANE_GAIN<Q_WVANE_GAIN>`
parameter. It converts the lean angle into degs/sec of yaw. A good value to start with is 1 (1 degree roll = 1 deg/sec yaw if :ref:`Q_PLT_Y_RATE<Q_PLT_Y_RATE>` =90). Higher values for gain or rate will make
the aircraft turn into the roll more quickly. If the overall correcting yaw rate is too high
then you can get instability and oscillation in yaw.

To cope with a small amount of trim in the aircraft there is an
additional parameter :ref:`Q_WVANE_ANG_MIN<Q_WVANE_ANG_MIN>` which
controls the minimum roll/pitch level before weathervaning will be
used. This defaults to one degree. If you find your aircraft starts
yawing even in no wind then you may need to raise this value.

Active weathervaning is only active in VTOL modes, and VTOL sections
of AUTO modes (such as VTOL takeoff and VTOL landing). It is not
active in QSTABILIZE and QHOVER modes as those are not position
controlled modes. It is active is QLOITER, QLAND and QRTL modes.

There are a number of additional parameters that can control when WeatherVaning is active (all are disabled by default):

- :ref:`Q_WVANE_HGT_MIN<Q_WVANE_HGT_MIN>`: above this height weathervaning is permitted
- :ref:`Q_WVANE_SPD_MAX<Q_WVANE_SPD_MAX>`: below this ground speed weathervaning is permitted
- :ref:`Q_WVANE_VELZ_MAX<Q_WVANE_VELZ_MAX>`: maximum climb or descent speed at which the vehicle will still attempt to weathervane
- :ref:`Q_WVANE_TAKEOFF<Q_WVANE_TAKEOFF>`: override weathervaning direction in auto takeoffs*
- :ref:`Q_WVANE_LAND<Q_WVANE_LAND>`: overide weathervaning directions in auto landings*

note:* not QLOITER take-offs and landings

.. note:: Weathervaning can be disabled or enabled by an :ref:`RC Aux Function Switch<common-auxiliary-functions>`, option "160"

Using the Forward or Tilt Motors to Help Position Holding
---------------------------------------------------------

In addition to active weathervaning, the QuadPlane code supports using
the forward motor or tilting the tilt rotors to hold the pitch level in VTOL flight modes.

In firmware versions prior to 4.5, to enable use of the forward motor(s)/tilting motors for position hold you need to set the :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>` parameter to a non-zero value.
This will be active in QLOITER, QRTL, QLAND and in AUTO mode when executing VTOL flight commands.This method uses the velocity controller.

In firmware versions 4.5 and later, use of this mechanism is no longer recommended and has been superseded by a new method which reduces the pitch forward experienced with the method activated using non-zero :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>`. This method is enabled by setting :ref:`Q_FWD_THR_GAIN<Q_FWD_THR_GAIN>` and :ref:`Q_FWD_THR_USE<Q_FWD_THR_USE>` to non-zero values.
This newer method uses the position controller to control the forward motor/tilt.

.. note:: Tailsitters do not have these options and :ref:`Q_VFWD_GAIN <Q_VFWD_GAIN>`, :ref:`Q_FWD_THR_GAIN<Q_FWD_THR_GAIN>` or :ref:`Q_FWD_THR_USE<Q_FWD_THR_USE>`, should be kept at the default value of 0.

Old Method (Q_VFWD_GAIN)
~~~~~~~~~~~~~~~~~~~~~~~~

The way it works is to look at two factors:

-  the navigation attitude pitch of the aircraft (which can be up to :ref:`Q_ANGLE_MAX<Q_ANGLE_MAX>`)
-  the difference between the desired forward velocity and the actual
   forward velocity

These are combined with the :ref:`Q_VFWD_GAIN<Q_VFWD_GAIN>` to ramp up and down the
throttle on the forward motor in order to minimize the attitude pitch
of the aircraft. That keeps the area of wing exposed to the wind
minimized which can reduce VTOL motor load.

A good value to start with for :ref:`Q_VFWD_GAIN<Q_VFWD_GAIN>` is 0.05. Higher values will
use the forward motor more aggressively. If the value is too high you
can get severe pitch oscillations.

As with active weathervaning, using the forward motor is only enabled
in position controlled VTOL modes. This means it is not enabled in
QSTABILIZE or QHOVER flight modes. It is available in QLOITER, QRTL,
QLAND and in AUTO mode when executing VTOL flight commands.

New Method (Q_FWD_THR_GAIN)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This method uses the position controller to control the forward motor thrust in combination with the navigation pitch angle which can be limited by the :ref:`Q_FWD_PIT_LIM<Q_FWD_PIT_LIM>`.

The :ref:`Q_FWD_THR_GAIN<Q_FWD_THR_GAIN>` parameter sets the gain from forward accel/tilt to forward throttle in Q modes. The Q modes this feature operates in is controlled by the :ref:`Q_FWD_THR_USE<Q_FWD_THR_USE>` parameter. Vehicles using separate forward thrust motors, eg quadplanes, should set this parameter to (all up weight) / (maximum combined thrust of forward motors) with a value of 2 being typical. Vehicles that tilt lifting rotors to provide forward thrust should set this parameter to (all up weight) / (weight lifted by tilting rotors) which for most aircraft can be approximated as (total number of lifting rotors) / (number of lifting rotors that tilt). When using this method of forward throttle control, the forward tilt angle limit is controlled by the :ref:`Q_FWD_PIT_LIM<Q_FWD_PIT_LIM>` parameter.

Note that you can also use reverse thrust on the forward motor using either method. If
your :ref:`THR_MIN <THR_MIN>` parameter is less than zero then reverse
thrust is available and the motor will use reverse thrust to slow down
or move backwards as needed. See the :ref:`reverse thrust <reverse-thrust-autolanding>` section for more details.

:ref:`Q_VFWD_ALT<Q_VFWD_ALT>`: when below this relative to home altitude, forward motor assist is disabled. This can be useful to keep the motor propeller from hitting the ground. Rangefinder height data is used when available.

.. note::
 Continuous tilt-rotor QuadPlanes will tilt motors up to :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` times :ref:`Q_TILT_MAX<Q_TILT_MAX>` to maintain position.
