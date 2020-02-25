.. _indoor-flying:

========================
Indoor Flying Guidelines
========================

This article provides important guidelines for flying your multicopter
inside.

.. warning::

   -  When flying indoor make sure you have plenty of space and follow
      safety procedures.
   -  GPS does not work indoors and needs to be disabled. (no exceptions)

Overview
--------

The main point note when flying indoors is that Global Positioning
Systems will not work. Even if you see that you have the correct number
of satellites and a low HDOP, this is due to multipathing of the
signals from the satellites. This means the single is being reflected to
the antenna via walls, windows and other surfaces from outside. If you
look at the position on a map you will see that the location will not
match your current location, or will be drifting, at time many meters or
even 1000s of metres from your location.

Stabilize
---------

:ref:`Stabilize <stabilize-mode>` mode does not use GPS, and as the
least problems, but the pilot will need good control of the copter.

Altitude Hold
-------------

:ref:`Altitude Hold <altholdmode>` mode use the barometer to hold a
specific altitude. The barometer relies on a constant air pressure.
Air-pressure in a room can change due to doors opening or closing. Also
climate control devices like fans and air-conditioning will also cause
pressure changes. The likely outcome is a sudden crash in the floor or
ceiling.

Sonar
-----

Downward facing :ref:`sonar or lidar <common-rangefinder-landingpage>` can help avoid with
sudden changes in altitude in AltHold mode causing crashes into the
floor or ceiling.

Safe Indoor Flying Dos
----------------------

-  Disable GPS in non-auto\*\* modes - set :ref:`AHRS_GPS_USE<AHRS_GPS_USE>` to 0
-  Disable GPS_FAILSAFE
-  Enable Battery_failsafe to LAND only or disable (not RTL) - set :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` parameter from Copter-3.6 or higher (for Copter-3.5 and lower use the FS_BATT_ENABLE parameter)
-  Enable Throttle Failsafe to LAND only or disable (not RTL or continue) - set :ref:`FS_THR_ENABLE <FS_THR_ENABLE>` = 0 or 3
-  Disable FENCE - set :ref:`FENCE_ENABLE <FENCE_ENABLE>` = 0
-  Use Sonar (if available)

Safe Indoor Flying Don'ts
-------------------------

-  Don't fly in small confined spaces. Use common sense, flying inside a
   warehouse with a high roof = OK, bedroom = not OK.
-  Don't use Auto\* modes

\* Auto Modes are ones that requires GPS i.e. Loiter, Position Hold,
Guided, Auto

\*\* Non-auto modes are Stabilize and Altitude Hold
