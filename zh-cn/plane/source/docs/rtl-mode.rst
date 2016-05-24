.. _rtl-mode:

===========================
RTL Mode (Return To Launch)
===========================

In RTL mode the plane will return to launch point (the point where the
plane first got a GPS lock) and loiter there until manual control is
regained (or it runs out of fuel!). As with :ref:`AUTO <auto-mode>` mode
you can also "nudge" the aircraft manually in this mode using stick
mixing. The target altitude for RTL mode is set using the
:ref:`ALT_HOLD_RTL <ALT_HOLD_RTL>` parameter in centimeters.

Alternatively, you may :ref:`configure the plane to return to a Rally Point <common-rally-points>`, rather than the launch point.

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
