.. _land-mode:

=========
Land Mode
=========

LAND Mode attempts to bring the copter straight down and has these
features:

-  descends at :ref:`LAND_SPD_HIGH_MS<LAND_SPD_HIGH_MS>`, if non-zero, (or :ref:`WP_SPD_DN <WP_SPD_DN>` if zero) using the regular Altitude Hold controller.
-  the pilot can reposition the vehicle using the pitch, roll, and yaw sticks unless the :ref:`LAND_REPOSITION <LAND_REPOSITION>` parameter is changed to "0", in which case position and yaw input are both ignored. The throttle stick has no effect by default, although high throttle can cancel landing if enabled via :ref:`PILOT_THR_BHV<PILOT_THR_BHV>`.
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

   Copter will recognise that it has landed when ALL of the following are true for
   approximately one second:

   - Motors are commanded to their lower limit by the vertical position controller
   - Throttle is at minimum 
   - No large angle is being requested (roll/pitch target < 15°)
   - No large angle error exists (attitude error < 30°)
   - The airframe is not accelerating downward > 1m/s/s ( >2 m/s/s if Weight on Wheels feature is enabled)
   - Vertical speed is within 1 m/s of zero ( within 2 m/s if Weight on Wheels feature is enabled)
   - If a healthy rangefinder is available, and rangefinder altitude is below 2m
   - Weight-on-Wheels (WoW) sensor (if present) confirms contact or is unknown

   Altitude above home is **not** used as a motor shutoff condition (except when a rangefinder is used,see above).
   The ``LAND_ALT_LOW_M`` parameter (default 10m) only controls the
   **descent speed transition** from :ref:`LAND_SPD_HIGH_MS<LAND_SPD_HIGH_MS>`
   to :ref:`LAND_SPD_MS<LAND_SPD_MS>` — it has no role in landing detection or disarming.

.. note::

   Traditional Heli uses heli-specific collective logic in place of
   "motors at lower limit" and "Throttle is at minimum" conditions above. In
   manual collective modes, the condition is met when collective is at or below
   :ref:`H_COL_LAND_MIN<H_COL_LAND_MIN>` or the collective stick is low.  In
   AUTOROTATE, it is met when collective is below the land-minimum value.  In
   altitude-controlled landing or descent, it is met when the
   collective controller is at its lower limit and is commanding a descent.
   The rotor will remain at or near the operating rotor speed as dictated by the
   rotor speed controller mode until Motor Interlock is disabled and disarming 
   occurs.

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
-  :ref:`LAND_REPOSITION<LAND_REPOSITION>` also controls whether the pilot's yaw
   stick is accepted while landing. This applies not only in LAND mode but
   also during the final descent stage of :ref:`RTL <rtl-mode>` and during
   LAND commands executed within a mission in :ref:`Auto mode <auto-mode>`.


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
