.. _follow-mode:

===========
Follow Mode
===========

..  youtube:: _g9SkK0IhRk
    :width: 100%

When in Follow mode, the vehicle will attempt to follow another vehicle (or anything publishing its position) at a specified offset.

The lead vehicle's position must be published to the vehicle in Follow mode using a :ref:`telemetry system <common-rangefinder-landingpage>`.  Telemetry radios which support mesh networking (e.g. :ref:`DroneBridge ESP32 <common-esp32-telemetry>`) are recommended.  See :ref:`Mult-Vehicle Flying <common-multi-vehicle-flying>` for more details.

Although Follow mode can use standard `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__ MAVLink messages from the lead vehicle, it generally works better if the lead vehicle publishes a `FOLLOW_TARGET <https://mavlink.io/en/messages/common.html#FOLLOW_TARGET>`__ MAVLink message. This can be done by running the `follow-target-send.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/follow-target-send.lua>`__ Lua script on the lead vehicle.

The following parameters can be used to tune Follow mode's performance:

-  :ref:`FOLL_ENABLE <FOLL_ENABLE>`: set to 1 to enable follow mode and refresh parameters
-  :ref:`FOLL_SYSID <FOLL_SYSID>`: MAVLink system id of the lead vehicle. This must be set explicitly; "0" means no lead vehicle has been selected and Follow is inactive
-  :ref:`FOLL_DIST_MAX <FOLL_DIST_MAX>`: if lead vehicle is more than this many meters away, give up on following and hold position
-  :ref:`FOLL_OFS_X <FOLL_OFS_X>`, :ref:`FOLL_OFS_Y <FOLL_OFS_Y>`, :ref:`FOLL_OFS_Z <FOLL_OFS_Z>`: 3D offset (in meters) from the lead vehicle
-  :ref:`FOLL_OFS_TYPE <FOLL_OFS_TYPE>`: set to 0 if offsets are North-East-Down, 1 if offsets are relative to lead vehicle's heading
-  :ref:`FOLL_YAW_BEHAVE <FOLL_YAW_BEHAVE>`: controls whether follow points in the same direction as lead vehicle or always towards it
-  :ref:`FOLL_POS_P <FOLL_POS_P>`: gain which controls how aggressively this vehicle moves towards lead vehicle (limited by :ref:`WP_SPD<WP_SPD>`)
-  :ref:`FOLL_ALT_TYPE <FOLL_ALT_TYPE>`: allows selecting whether to use lead vehicle's relative-to-home or relative-to-sea-level altitude
-  :ref:`FOLL_OPTIONS<FOLL_OPTIONS>`: set bit 0 to "1" to enable the :ref:`common-mount-targeting` to follow the target vehicle.
-  :ref:`FOLL_ACCEL_NE <FOLL_ACCEL_NE>`, :ref:`FOLL_JERK_NE <FOLL_JERK_NE>`: Acceleration and jerk limits applied to the lead vehicle's estimated horizontal position, velocity and acceleration
-  :ref:`FOLL_ACCEL_D <FOLL_ACCEL_D>`, :ref:`FOLL_JERK_D <FOLL_JERK_D>`: Acceleration and jerk limits applied to the lead vehicle's estimated vertical position, velocity and acceleration
-  :ref:`FOLL_ACCEL_H <FOLL_ACCEL_H>`, :ref:`FOLL_JERK_H <FOLL_JERK_H>`: Acceleration and jerk limits applied to the lead vehicle's estimated heading and yaw rotation rate
-  :ref:`FOLL_TIMEOUT <FOLL_TIMEOUT>`: Timeout in seconds.  If no messages are received from the lead vehicle within this many seconds, the follower will give up and hold position
-  :ref:`WP_SPD <WP_SPD>`, :ref:`WP_SPD_UP <WP_SPD_UP>`, :ref:`WP_SPD_DN <WP_SPD_DN>`, :ref:`WP_ACC <WP_ACC>`, :ref:`WP_ACC_Z <WP_ACC_Z>`: horizontal and vertical speed and acceleration limits for the following vehicle

Choosing the Altitude Type
==========================

:ref:`FOLL_ALT_TYPE <FOLL_ALT_TYPE>` selects which altitude field from the lead vehicle is
used as the follow target, and getting it wrong is a common cause of the follower sitting at
an unexpected height.

- "1" (relative, the default on Copter) uses the lead vehicle's altitude above **its own** home.
  Choose this if both vehicles take off from places whose home altitudes are physically the
  same, and the EKF altitude source is the default barometer (i.e. ``EK3_SRCx_POSZ`` = Baro).
  Because both vehicles then measure height from the same physical datum, barometric drift and
  GPS altitude error largely cancel out and the follower holds the commanded offset accurately.
- "0" (absolute) uses the lead vehicle's altitude above mean sea level. Choose this in any other
  situation, in particular when the vehicles take off from different elevations, or when either
  vehicle uses a non-barometric altitude source. Note that absolute altitude carries the full
  error of each vehicle's own altitude estimate, so a constant vertical offset between the
  vehicles is normal.
- "3" (terrain) converts the lead vehicle's altitude to a height above terrain, so the follower
  matches the lead vehicle's clearance above ground rather than its altitude. This requires
  terrain data (see :ref:`common-terrain-following`) to be available for the lead vehicle's
  position. If it is not, the target update is discarded and the follower will stop tracking,
  so only use this option where terrain data is known to be loaded.

.. note:: The lead vehicle's home altitude is not transmitted, so ArduPilot cannot detect a
   mismatch between the two vehicles' home altitudes. If the follower consistently flies too
   high or too low by roughly the difference in take-off elevations, "relative" is the wrong
   choice for that setup.

Changing the Lead Vehicle
=========================

:ref:`FOLL_SYSID<FOLL_SYSID>` may be changed while flying, for example by a Lua script switching between two beacons. The position, velocity and heading held for the previous lead vehicle are discarded as soon as the parameter changes, so the follower has no target and holds its current position until the newly selected vehicle is heard from. Tracking then restarts from that vehicle's position, rather than being smoothed across from the old one.

The type of position message in use is also reset by the change, so if the previous lead vehicle was sending ``FOLLOW_TARGET``, ``GLOBAL_POSITION_INT`` from the new lead vehicle is accepted immediately.

Setting :ref:`FOLL_SYSID<FOLL_SYSID>` to "0" deselects the lead vehicle altogether, and nothing is followed until a non-zero system id is set. Earlier firmware treated "0" as "follow the first vehicle seen" and wrote that vehicle's system id into the parameter; it no longer does either, so the lead vehicle must always be selected explicitly.
