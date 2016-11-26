.. _loiter-mode:

===========
LOITER Mode
===========

In LOITER mode the plane will circle around the point where you started
the loiter, holding altitude at the altitude that you entered loiter in.
The radius of the circle is controlled by the WP_LOITER_RAD parameter,
but is also limited by your ``NAV_ROLL_CD`` limit, and your
`:ref:`NAVL1_PERIOD`` navigation tuning. As with `Return To Launch (RTL) <rtl-mode>` and :ref:`AUTO <auto-mode>`
mode you can "nudge" the plane while in LOITER using stick mixing, if
enabled.

.. warning::

   "Home" position is always supposed to be your Plane's actual
   GPS takeoff location:

   #. It is very important to acquire GPS lock before arming in order for
      RTL, Loiter, Auto or any GPS dependent mode to work properly.
   #. For Plane the home position is the postion of the Plane when you
      first get GPS lock whether it was armed or not.

      -  This means if you execute an RTL in Plane, it will return to the
         location where it was when it first acquired GPS lock.
      -  For Plane: Plug in the battery and let it acquire GPS lock where
         you want it to return to: (Not the Pits).
