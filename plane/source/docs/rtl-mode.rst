.. _rtl-mode:

===========================
RTL Mode (Return To Launch)
===========================

In RTL mode the plane will return to its home location (the point where the
plane armed - assuming it had GPS lock) and loiter there until given
alternate instructions (or it runs out of fuel!). As with :ref:`AUTO <auto-mode>` mode
you can also "nudge" the aircraft manually in this mode using stick
mixing (assuming the RTL was not a result of RC failsafe).

The target altitude for RTL mode is set using the :ref:`ALT_HOLD_RTL <ALT_HOLD_RTL>` parameter in centimeters. When initiated below :ref:`ALT_HOLD_RTL <ALT_HOLD_RTL>`, Plane will immediately climb at maximum allowable rate to reach that altitude, if above, it will descend in a linear manner versus distance to home, reaching that altitude at the home loiter point.

The loiter radius at home is determined by :ref:`RTL_RADIUS<RTL_RADIUS>`, if it's non-zero, otherwise :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` is used. Clock-wise or Counter-Clock-wise circling can be set by positive or negative values for the radius.

Additionally, in firmware versions 4.0.6 and later, by setting :ref:`RTL_CLIMB_MIN<RTL_CLIMB_MIN>` to a non-zero value, the plane will climb that many meters, limited in bank angle by :ref:`LEVEL_ROLL_LIMIT<LEVEL_ROLL_LIMIT>` degrees until the additional altitude is reached, then begin the above return to home action from that altitude.

Alternatively, you may :ref:`configure the plane to return to a Rally Point <common-rally-points>`, rather than the home location.

.. warning::

   "Home" position is always supposed to be your Plane's actual
   GPS takeoff location:

   #. It is very important to acquire GPS lock before arming in order for
      RTL, Loiter, Auto or any GPS dependent mode to work properly.
   #. For Plane the home position is initially established at the time the
      plane acquires its GPS lock. It is then continuously updated as long as
      the autopilot is disarmed.

      - This means if you execute an RTL in Plane, it will return to the
	location where it was when it was armed - assuming it had
	acquired GPS lock.
      - Consider the use of :ref:`Rally Points <common-rally-points>` to
	avoid returning directly to your arming point on RTL
