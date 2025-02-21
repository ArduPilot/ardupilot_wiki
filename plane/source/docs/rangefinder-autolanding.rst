.. _rangefinder-autolanding:

===================================
Using a Rangefinder for Autolanding
===================================

If you have :ref:`fitted a rangefinder <common-rangefinder-landingpage>`
to your aircraft then you can use it for much more accurate landing
control. To allow the rangefinder to be used for landing you need to set
the :ref:`RNGFND_LANDING <RNGFND_LANDING>` parameter to 1. The first rangefinder
with :ref:`RNGFND_LND_ORNT<RNGFND_LND_ORNT>` orientation found, will be used. Normally, this will be "down" (25) for most planes, but Tailsitter Quadplanes would normally use "back" (4) so that the rangefinder is pointing at the ground during a tailsitter landing with its nose vertical.

When using a rangefinder for landing the altitude given by the
rangefinder is used only in the landing approach and to determine the
flare point, and is designed to allow the aircraft to more accurately
follow the glide slope and to flare at the right time.

.. note::

   The effectiveness of a rangefinder can depend on the surface you
   are flying over, so it is a good idea to do some low passes in a flight
   mode such as FBWA first, then examine the logs to check that the
   rangefinder is working correctly.

Also note that if you have a longer range rangefinder then it is a very
good idea to set the minimum range of the rangerfinder well above zero.
For example, the PulsedLight Lidar has a typical range of over 40
meters, and when it gets false readings it tends to read ranges of less
than 1 meter. And setting :ref:`RNGFND1_MIN <RNGFND1_MIN>` to 150 , if its the first system rangefinder, will discard any rangefinder readings below 1.5 meters, and will
greatly improve the robustness of the Lidar for landing.

If the autopilot has a good rangefinder (:ref:`such as LIDAR <common-rangefinder-lidarlite>`) then you can safely choose quite small numbers for :ref:`LAND_FLARE_SEC<LAND_FLARE_SEC>` and :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>`, and flare closer to the ground than with the default values. 
That will generally produce a better landing. 

A value for :ref:`LAND_FLARE_SEC<LAND_FLARE_SEC>` of 1.5 and :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` of 2 is a good place to start with a LiDAR. 

Impact of Glide Slope Terrain variation
=======================================

If using rangefinder during landing, the rangefinder readings, once engaged (ie when getting good readings below the :ref:`RNGFND1_MAX<RNGFND1_MAX>` altitude on approach), will continuosly adjust the glide slope target altitudes as it approaches. If the terrain is irregular, or there is a deep ditch before the runway threshold, using the rangefinder can actually be worse than just using the barometeric altitude.

In these cases, you may wish to lower the :ref:`RNGFND1_MAX<RNGFND1_MAX>` altitude to a value closer to the expected vehicle altitude at the end of the runway so that it engages and is used shortly before the flare.

Alternatively, if the terrain is sloping toward or away from the runway, you may use the :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>` parameter (bit 1(all modes) or 3(AUTO mode only) set to provide additional corrections to the rangefinder readings for the difference between landing point terrain altitude and current approach altitude above terrain to help prevent landing overshoots and undershoots. Be sure to carefully test for your particular situation and be ready to abort the autolanding if it does not look safe.