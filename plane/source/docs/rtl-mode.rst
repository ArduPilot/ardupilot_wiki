.. _rtl-mode:

===========================
RTL Mode (Return To Launch)
===========================

In RTL mode the plane will return to its home location (the point where the
plane armed - assuming it had GPS lock) and loiter there until given
alternate intructions (or it runs out of fuel!). As with :ref:`AUTO <auto-mode>` mode
you can also "nudge" the aircraft manually in this mode using stick
mixing. The target altitude for RTL mode is set using the
:ref:`ALT_HOLD_RTL <ALT_HOLD_RTL>` parameter in centimeters.

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
