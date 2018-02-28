.. _smartrtl-mode:

==============
Smart RTL Mode
==============

Smart RTL was added to the master 3.6-dev as part of a Google Summer of Code project in 2017.  It was added as a failsafe option in January 2018. Smart RTL tracks the copter’s course of flight, to include position and altitude. It will prune out unnecessary circles and loops, to continuously form a clean and efficient path home over a known course. So if your course of flight was around and over obstacles, Smart RTL will follow that path home, avoiding those same obstacles. But it will do so without flying in unnecessary circles and loops.
When switched into Smart RTL (aka Smart Return-To-Launch), like regular RTL, the vehicle will attempt to return to the position where it was last armed.

The "Smart" part of this mode is that it will retrace its path to return to home instead of returning directly home.  This can be useful if there obstacles between the vehicle and the home position.

.. note::

   The vehicle must have a good position estimate (LEDs should be green) when the vehicle is armed or SmartRTL will be disabled.  If the vehicle is armed without a good position estimate, "SmartRTL deactivated: bad position" will be sent to the ground station.

The path used to return home is captured in a buffer as the vehicle flies around in any other mode.  The path is "simplified" (meaning curved paths are turned into a series of straight lines) and "pruned" (meaning loops are removed).  The buffer is of a limited size (see below) and once it is full, "SmartRTL deactivated: buffer full" will appear on the ground stations's HUD and the user will no longer be able to switch into this mode.

..  youtube:: gXfBmFn_JEU
    :width: 100%

-  :ref:`RTL_SPEED <RTL_SPEED>` can be used to set the speed (in meters/second) at which the vehicle will return to home.  By default this parameter is zero meaning the :ref:`WP_SPEED <WP_SPEED>` or :ref:`CRUISE_SPEED <CRUISE_SPEED>` parameter values will be used.
-  :ref:`SRTL_ACCURACY <SRTL_ACCURACY>` controls the accuracy (in meters) of the simplification and pruning performed.  The simplify algorithm will turn curved paths into straight lines but the line should never be more than this distance from the original path.  The pruning algorithm will cut paths that come within this many meters of each other.
-  :ref:`SRTL_POINTS <SRTL_POINTS>` controls the maximum number of points that can be stored.  Each additional 100 points requires about 3k of RAM.
