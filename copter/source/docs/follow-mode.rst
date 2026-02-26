.. _follow-mode:

===========
Follow Mode
===========

..  youtube:: _g9SkK0IhRk
    :width: 100%

When switched into Follow, the vehicle will attempt to follow another vehicle (or anything publishing its position) at a specified offset.  The vehicle lead vehicle's position must be published to the vehicle in Follow mode using a telemetry system.

The altitude is maintained with the altitude hold controller so the vehicle will attempt to hold its current altitude when the sticks are placed with 10% of mid-throttle. It will climb or descend at up to 2.5m/s (this speed is adjustable with the :ref:`PILOT_SPEED_UP<PILOT_SPEED_UP>` and :ref:`PILOT_SPEED_DN<PILOT_SPEED_DN>` parameters). The acceleration used to establish these speeds is set by :ref:`PILOT_ACCEL_Z<PILOT_ACCEL_Z>`.

The following parameters can be used to tune Follow Mode's performance:

-  :ref:`FOLL_ENABLE <FOLL_ENABLE>`: set to 1 to enable follow mode and refresh parameters
-  :ref:`FOLL_SYSID <FOLL_SYSID>`: MAVLink system id of lead vehicle ("0" means follow the first vehicle "seen")
-  :ref:`FOLL_DIST_MAX <FOLL_DIST_MAX>`: if lead vehicle is more than this many meters away, give up on following and hold position
-  :ref:`FOLL_OFS_X <FOLL_OFS_X>`, :ref:`FOLL_OFS_Y <FOLL_OFS_Y>`, :ref:`FOLL_OFS_Z <FOLL_OFS_Z>`: 3D offset (in meters) from the lead vehicle
-  :ref:`FOLL_OFS_TYPE <FOLL_OFS_TYPE>`: set to 0 if offsets are North-East-Down, 1 if offsets are relative to lead vehicle's heading
-  :ref:`FOLL_YAW_BEHAVE <FOLL_YAW_BEHAVE>`: controls whether follow points in the same direction as lead vehicle or always towards it
-  :ref:`FOLL_POS_P <FOLL_POS_P>`: gain which controls how aggressively this vehicle moves towards lead vehicle (limited by :ref:`WPNAV_SPEED <WPNAV_SPEED>`)
-  :ref:`FOLL_ALT_TYPE <FOLL_ALT_TYPE>`: allows selecting whether to use lead vehicle's relative-to-home or relative-to-sea-level altitude
-  :ref:`FOLL_OPTIONS<FOLL_OPTIONS>`: set bit 0 to "1" to enable the :ref:`common-mount-targeting` to follow the target vehicle.
