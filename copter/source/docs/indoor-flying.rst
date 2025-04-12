.. _indoor-flying:

========================
Indoor Flying Guidelines
========================

This article provides guidelines for flying your multicopter inside without any position or velocity source (e.g. no GPS, no optical flow).
For indoor flight using a non-GPS position or velocity source please refer to the :ref:`Non-GPS Navigation <common-non-gps-navigation-landing-page>` page.

.. warning::

   -  When flying indoors make sure you have plenty of space and follow
      safety procedures.
   -  GPS does not work indoors and needs to be disabled (no exceptions)

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

:ref:`Stabilize <stabilize-mode>` mode does not use GPS, and has the
least problems, but the pilot will need good control of the copter.

Altitude Hold
-------------

:ref:`Altitude Hold <altholdmode>` mode use the barometer to hold a
specific altitude. The barometer relies on a constant air pressure.
Air-pressure in a room can change due to doors opening or closing. Also
climate control devices like fans and air-conditioning will also cause
pressure changes. The likely outcome is a sudden crash in the floor or
ceiling.

Sonar or Lidar
--------------

Using a downward facing :ref:`sonar or lidar <common-rangefinder-landingpage>` when flying in :ref:`AltHold <altholdmode>` can help avoid sudden changes in altitude causing crashes into the floor or ceiling.  See :ref:`Surface Tracking <terrain-following-manual-modes>` for more details.

Safe Indoor Flying Dos
----------------------

-  Disable the GPS by setting :ref:`GPS1_TYPE<GPS1_TYPE>` to 0
-  Configure the :ref:`battery failsafe <failsafe-battery>` to "Land" by setting :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` = 1
-  Configure the :ref:`radio failsafe <radio-failsafe>` to "Enabled always Land" by setting :ref:`FS_THR_ENABLE <FS_THR_ENABLE>` = 3
-  Disable the :ref:`fence <common-geofencing-landing-page>` by setting :ref:`FENCE_ENABLE <FENCE_ENABLE>` = 0
-  Use a downward facing sonar or lidar (if available)

Safe Indoor Flying Don'ts
-------------------------

-  Don't fly in small confined spaces. Use common sense, flying inside a
   warehouse with a high roof = OK, bedroom = not OK.
-  Don't use Auto\* modes

\* Autonomous and semi-autonomous modes requires GPS (e.g. :ref:`Loiter <loiter-mode>`, :ref:`Guided <ac2_guidedmode>`, :ref:`Auto <auto-mode>`, :ref:`RTL <rtl-mode>`)

\*\* Non-autonomous modes include :ref:`Stabilize <stabilize-mode>` and :ref:`AltHold <altholdmode>`
