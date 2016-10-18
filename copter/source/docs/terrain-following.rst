.. _terrain-following:

=================
Terrain Following
=================

Copter 3.4 (and higher) support "terrain following" in :ref:`AUTO <auto-mode>`, :ref:`Guided <ac2_guidedmode>`, :ref:`RTL <rtl-mode>` and :ref:`Land <land-mode>` flight modes.  This feature allows the vehicle to climb or descend to maintain a specified distance above the terrain using either a :ref:`downward facing Lidar or Sonar <common-rangefinder-landingpage>` or from terrain altitude data provided by the ground station using a mapping service such as Google maps.  Details of how the Google maps data is used can be found on the :ref:`plane terrain following page <common-terrain-following>`

..  youtube:: mT67QOAxuG8
    :width: 100%

Setting up a Mission to use Terrain data
----------------------------------------
-  If relying on a :ref:`downward facing LIDAR ensure it is setup as described here <common-rangefinder-landingpage>`
-  If using GCS provided terrain data set the :ref:`TERRAIN_ENABLE <TERRAIN_ENABLE>` parameter to 1
-  Using a recent version of Mission Planner (or other GCS that supports terrain following) on the Flight Plan screen, set the altitude type to "Terrain".  Once set all mission commands that include an "Alt" fields will be interpreted as altitudes-above-terrain.
-  Upload the mission to the vehicle and execute the mission as you normally would in :ref:`AUTO <auto-mode>`

   .. image:: ../images/terrain_mission.png
       :target: ../_images/terrain_mission.png
       :width: 500px


Using Terrain Altitude during RTL and Land
------------------------------------------
Set the :ref:`TERRAIN_FOLLOW <TERRAIN_FOLLOW>` parameter to 1 to enable using terrain data in :ref:`RTL <rtl-mode>` and :ref:`Land <land-mode>` flight modes.  If set the vehicle will interpret the :ref:`RTL_ALT <RTL_ALT>` as an altitude-above-terrain meaning it will generally climb over hills on it's return path to home.  Similarly Land will slow to the :ref:`LAND_SPEED <LAND_SPEED>` (normally 50cm/s) when it is 10m above the terrain (instead of 10m above home).
Currently setting this parameter is not recommended because of the edge case mentioned below involving the somewhat unlikely situation in which the vehicle is unable to retrieve terrain data during the :ref:`RTL <rtl-mode>`.  In these cases the :ref:`RTL_ALT <RTL_ALT>` will be interpreted as an alt-above home. 

Failsafe in case of no Terrain data
------------------------------------------
If the vehicle is executing a mission command that requires terrain data but it is unable to retrieve terrain data for two seconds (normally because the range finder fails, goes out of range or the Ground Station is unable to provide terrain data) the vehicle will switch to RTL mode.  Note that because it does not immediately have access to terrain data in this situation it will perform a normal RTL interpreting the :ref:`RTL_ALT <RTL_ALT>` as an altitude-above-home regardless of whether :ref:`TERRAIN_FOLLOW <TERRAIN_FOLLOW>` has been set to "1" or not.

Terrain Spacing and Accuracy
----------------------------

The :ref:`TERRAIN_SPACING <TERRAIN_SPACING>` parameter controls the size of the grid used when requesting terrain altitude from the Ground Station (it is not used if using a Lidar). This is 100m by default but reducing to 30 may provide better accuracy at the expense of more telemetry traffic between the GCS and Flight controller.  It is recommended that you use a :ref:`TERRAIN_SPACING <TERRAIN_SPACING>` of at least 30 meters.

If the ground station does not have terrain data available at the resolution requested by the aircraft then the ground station will interpolate as necessary to provide the requested grid size.

Terrain Accuracy
----------------

The accuracy of the SRTM database varies over the surface of the earth.  Typical accuracy is around 10 to 20 meters, but some developers have noticed inaccuracies of 35m.  This makes terrain following suitable for aircraft that are flying at altitudes of 60 meters or more.  For very accurate terrain following at lower altitudes it is recommended to use a :ref:`downward facing Lidar or Sonar <common-rangefinder-landingpage>`.

Example mission at 2m using Lidar
---------------------------------

..  youtube:: r4RBP0_LQ5Y
    :width: 100%
