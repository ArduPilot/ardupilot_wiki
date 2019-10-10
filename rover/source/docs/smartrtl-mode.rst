.. _smartrtl-mode:

==============
Smart RTL Mode
==============

..  youtube:: V0bolTN8QRM
    :width: 100%

When switched into Smart RTL, like regular RTL, the vehicle will attempt to return home.  The "Smart" part of this mode is that it will retrace a safe path home instead of returning directly home.  This can be useful if there are obstacles between the vehicle and the home position. Once at the destination it will hold position if a surface vehicle, or loiter/circle around the destination if it is a boat.

.. note::

   The vehicle must have a good position estimate (LEDs should be green) when the rover is armed or SmartRTL will be disabled.  If the vehicle is armed without a good position estimate, "SmartRTL deactivated: bad position" will be sent to the ground station.

The path used to return home is captured in a buffer as the vehicle flies around in any other mode.  The path is "simplified" (meaning curved paths are turned into a series of straight lines) and "pruned" (meaning loops are removed).  The buffer is of a limited size (see below). When it gets close to being full, "SmartRTL low on space" will be sent to the ground station as an information message and once it is full, "SmartRTL deactivated: buffer full" will appear on the ground stations's HUD and the user will no longer be able to switch into this mode. To avoid SmartRTL being deactivated, you can switch into the mode temporarily, bleeding off path points for a while when returning, then exit the mode and continue until the space becomes low again.

-  :ref:`RTL_SPEED <RTL_SPEED>` can be used to set the speed (in meters/second) at which the vehicle will return to home.  By default this parameter is zero, meaning the :ref:`WP_SPEED <WP_SPEED>` parameter values will be used, or :ref:`CRUISE_SPEED <CRUISE_SPEED>` in versions preceeding Rover 4.0, if :ref:`WP_SPEED <WP_SPEED>` is also 0.
-  :ref:`SRTL_ACCURACY <SRTL_ACCURACY>` controls the accuracy (in meters) of the simplification and pruning performed.  The simplify algorithm will turn curved paths into straight lines but the line should never be more than this distance from the original path.  The pruning algorithm will cut paths that come within this many meters of each other.
-  :ref:`SRTL_POINTS <SRTL_POINTS>` controls the maximum number of points that can be stored.  Each additional 100 points requires about 3k of RAM. The maximum number of points allowed is 500.
