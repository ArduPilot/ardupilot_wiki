.. _training-mode:

=============
TRAINING Mode
=============

TRAINING mode is ideal for teaching students manual R/C control. It
gives users complete control over the rudder and throttle, but clips the
maximum roll and maximum/minimum pitch to certain limits which cannot be
exceeded. TRAINING mode also restricts plane
roll to the :ref:`Stall Prevention <stall-prevention>` roll limits.

More specifically:

-  If the roll is less the :ref:`ROLL_LIMIT_DEG <ROLL_LIMIT_DEG>`
   parameter than the pilot has manual roll control. If the plane tries
   to roll past that limit then the roll will be held at that limit. The
   plane will not automatically roll back to level flight, but it will
   prevent the pilot from rolling past the limit. The same applies to
   pitch - the pilot has manual pitch control until the
   :ref:`PTCH_LIM_MIN_DEG <PTCH_LIM_MIN_DEG>` or :ref:`PTCH_LIM_MAX_DEG <PTCH_LIM_MAX_DEG>`
   limits are reached, at which point the plane won't allow the pitch to
   go past those limits.
-  When turning, the autopilot will monitor the demanded bank angle and
   airspeed and work out if there is a sufficient margin above the stall
   speed to turn at the demanded bank angle. If not the bank angle is
   limited to the safe value. The stall prevention system will always
   allow a bank of at least 25 degrees (to ensure you can still
   manoeuvre if your airspeed estimate is badly off).
-  The rudder and throttle are both completely under manual control.

.. note:: the :ref:`MAN_EXPO_ROLL<MAN_EXPO_ROLL>`, :ref:`MAN_EXPO_PITCH<MAN_EXPO_PITCH>`, and :ref:`MAN_EXPO_RUDDER<MAN_EXPO_RUDDER>` parameters will apply exponential to the stick inputs, if non-zero, in this mode. This is for users with transmitters which do not provide this function and desire to "soften" stick feel around neutral.