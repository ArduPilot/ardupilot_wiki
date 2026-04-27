.. _land-mode:

=========
Land Mode
=========

LAND Mode attempts to bring the copter straight down and has these
features:

-  descends at :ref:`LAND_SPD_HIGH_MS<LAND_SPD_HIGH_MS>`, if non-zero, (or :ref:`WP_SPD_DN <WP_SPD_DN>` if zero) using the regular Altitude Hold controller.
-  the pilot can reposition the vehicle using the pitch and roll sticks unless the :ref:`LAND_REPOSITION <LAND_REPOSITION>` parameter is changed to "0". The throttle stick has no effect by default, although high throttle can cancel landing if enabled via :ref:`PILOT_THR_BHV<PILOT_THR_BHV>`.
-  if a rangefinder is being used, or :ref:`TERRAIN_ENABLE<TERRAIN_ENABLE>` =1 and terrain data is available, the descent speed will then switch to :ref:`LAND_SPD_MS<LAND_SPD_MS>` at :ref:`LAND_ALT_LOW_M<LAND_ALT_LOW_M>` altitude (default is 10m) above ground until landing occurs. If neither terrain data or rangefinder data is available, then altitude above HOME will be used for the speed switch point (if reached).

   .. image:: ../images/Land_DescentSpeed1.png
       :target: ../_images/Land_DescentSpeed1.png

-  below 10m the copter should descend at the rate specified in the
   :ref:`LAND_SPD_MS<LAND_SPD_MS>` parameter which defaults to 50cm/s.

   .. image:: ../images/Land_DescentSpeed2.png
       :target: ../_images/Land_DescentSpeed2.png

-  Upon reaching the ground the copter will automatically shut-down the
   motors and disarm the copter if the pilot's throttle is at minimum.

.. note::

    Copter will recognise that it has landed if the motors are being commanded to be at low
    level by the vertical position controller, its vertical speed is close to zero (within ±1 m/s by default), 
    is not accelerating for one second, and other internal landing-detection checks, such as attitude-related checks, 
    are also satisfied.  It does not use the altitude to decide whether to shut off the
    motors, except that when a healthy rangefinder is being used, the copter must be within 2m of the ground.

.. note:: For Traditional Heli, the low motor check in the above landing detection algorithm is replaced with a check that Collective output is below
   mid-position (controlled by the vertical position controller, ie in descent). The rotor still may be at governor speed up until Motor Interlock is removed and  disarming occurs.

.. note:: Using a Weight on Wheels (WoW) switch will increase the descent rate and
    accelerometer ranges that are acceptable for landing detection. This
    feature is enabled anytime the LGR_WOW_PIN is not disabled.


-  If the copter appears to bounce or balloon back up a couple of times
   before settling down and turning the props off, try lowering the
   :ref:`LAND_SPD_MS<LAND_SPD_MS>` parameter a bit.
-  If the vehicle has GPS lock the landing controller will attempt to
   control its horizontal position, but the pilot can adjust the target
   horizontal position just as in Loiter mode, unless the 
   :ref:`LAND_REPOSITION<LAND_REPOSITION>` parameter is set to 0.
-  If the vehicle does not have GPS lock the horizontal control will be
   as in stabilize mode, so the pilot can control the roll and pitch lean
   angle of the copter, unless the :ref:`LAND_REPOSITION<LAND_REPOSITION>` parameter is set to 0.


.. warning::

    In any Alt Hold based mode including: Alt Hold, Loiter,
    Auto, Auto NAV_Land or RTL if your copters operation becomes erratic when you
    are close to the ground or landing (and also if any auto landing
    procedure results in bouncing or failure to turn off motors properly
    after landing) you probably have the autopilot situated such that
    its barometer (altimeter) is being affected by the pressure created by
    the copters prop-wash against the ground.



-  This is easily verified by looking at the Altimeter reading in your
   logs and seeing if it spikes or oscillates when near the ground.
-  If this is a problem, move the autopilot out of prop wash
   effect or shield it with an appropriately ventilated enclosure.
-  Success can be verified by flight test and by log results.

