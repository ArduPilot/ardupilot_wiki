.. _loiter-mode:

===========
LOITER Mode
===========

In LOITER mode the plane will circle around the point where you started
the loiter, holding altitude at the altitude that you entered loiter in.
The radius of the circle is controlled by the ``WP_LOITER_RAD`` parameter,
but is also limited by your ``NAV_ROLL_CD`` limit, and your ``NAVL1_PERIOD``
navigation tuning. As with :ref:`Return To Launch (RTL) <rtl-mode>` and
:ref:`AUTO <auto-mode>` mode you can "nudge" the plane while in LOITER
using stick mixing, if enabled.

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
