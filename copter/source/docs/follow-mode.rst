.. _follow-mode:

===========
Follow Mode
===========

..  youtube:: _g9SkK0IhRk
    :width: 100%

When switched into Follow, the vehicle will attempt to follow another vehicle (or anything publishing its position) at a specified offset.  The vehicle lead vehicle's position must be published to the vehicle in Follow mode using a telemetry system.

.. note::

   Follow is available in Copter-3.6 and Rover-3.5 (and higher)

The following parameters can be used to tune FlowHold's performance:

-  :ref:`FOLL_ENABLE <FOLL_ENABLE>` : set to 1 to enable follow mode and refresh parameters
-  :ref:`FOLL_SYSID <FOLL_SYSID>` : MAVLink system id of lead vehicle ("0" means follow the first vehicle "seen")
-  :ref:`FOLL_DIST_MAX <FOLL_DIST_MAX>` : if lead vehicle is more than this many meters away, give up on following and hold position
-  :ref:`FOLL_OFS_X <FOLL_OFS_X>`, :ref:`FOLL_OFS_Y <FOLL_OFS_Y>`, :ref:`FOLL_OFS_Z <FOLL_OFS_Z>` : 3D offset (in meters) from the lead vehicle
-  :ref:`FOLL_OFS_TYPE <FOLL_OFS_TYPE>` : set to 0 if offsets are North-East-Down, 1 if offsets are relative to lead vehicle's heading
-  :ref:`FOLL_YAW_BEHAVE <FOLL_YAW_BEHAVE>` : controls whether follow points in the same direction as lead vehicle or always towards it
-  :ref:`FOLL_POS_P <FOLL_POS_P>` : gain which controls how aggressively this vehicle moves towards lead vehicle (limited by :ref:`WPNAV_SPEED <WPNAV_SPEED>`)
-  :ref:`FOLL_ALT_TYPE <FOLL_ALT_TYPE>` : allows selecting whether to use lead vehicle's relative-to-home or relative-to-sea-level altitude
