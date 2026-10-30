.. _follow-mode:

===========
Follow Mode
===========

..  youtube:: _g9SkK0IhRk
    :width: 100%

When switched into Follow, the vehicle will attempt to follow another vehicle (or anything publishing its position) at a specified offset.  The lead vehicle's position must be published to the vehicle in Follow mode using a telemetry system as shown below. QGroundControl's application (QGC) for Android phones and computers has this feature in beta testing also, so instead of following a vehicle, it will follow the ground control station, similar to FOLLOW ME mode in Copter.

In its simplest form, each vehicle has a telemetry radio with the same NETID, on one of their telemetry ports.

.. image:: ../images/Follow1.jpg
   :target: ../_images/Follow1.jpg

Vehicle A merely enters FOLLOW mode and it will attempt to follow Vehicle B, and vice-versa.

If Vehicle A or B requires Ground Station Control, an additional Telemetry radio on a different NETID (YY) can be installed on another telemetry port on the vehicle controller, paired with the GCS's radio on that same NETID.

.. image:: ../images/Follow2.jpg
   :target: ../_images/Follow2.jpg

If using QGroundControl ensure the Application setting under the General tab ``Stream GCS Position`` is set to "When in Follow Me Flight Mode" or "Always".  Next connect to the vehicle and switch to FOLLOW Mode and it will then follow the computer/phone running QGC.

.. image:: ../images/follow-mode-qgc-setting.png
   :target: ../_images/follow-mode-qgc-setting.png

The following parameters can be used to tune Follow mode's performance:

-  :ref:`FOLL_ENABLE <FOLL_ENABLE>`: set to 1 to enable follow mode and refresh parameters.
-  :ref:`FOLL_SYSID <FOLL_SYSID>`: MAVLink system id of the lead vehicle. This must be set explicitly; "0" means no lead vehicle has been selected and Follow is inactive.
-  :ref:`FOLL_DIST_MAX <FOLL_DIST_MAX>`: if lead vehicle is more than this many meters away, give up on following and hold position (loiter if boat, stop if ground vehicle).
-  :ref:`FOLL_OFS_X <FOLL_OFS_X>`, :ref:`FOLL_OFS_Y <FOLL_OFS_Y>`, :ref:`FOLL_OFS_Z <FOLL_OFS_Z>` (not used in Rover) : 3D offset (in meters) from the lead vehicle. If they are zero, then the current vehicle's offset from the follow target at time of mode entry is used every time. These offsets can be altered via Mavlink and will take effect immediately. However, if they were originally zero and changed during FOLLOW MODE rather than during another mode, they will be reset to zero for the next entry into FOLLOW mode, until a reboot occurs and then the changed offsets will be restored. 
-  :ref:`FOLL_OFS_TYPE <FOLL_OFS_TYPE>`: set to 0 if offsets are North-East (NED), 1 if offsets are relative to lead vehicle's heading, see diagrams below.
-  :ref:`FOLL_POS_P <FOLL_POS_P>`: gain which controls how aggressively this vehicle moves towards lead vehicle (limited by :ref:`WP_SPD<WP_SPD>`)

Changing the Lead Vehicle
=========================

:ref:`FOLL_SYSID<FOLL_SYSID>` may be changed while driving. The position, velocity and heading held for the previous lead vehicle are discarded as soon as the parameter changes, so the follower has no target and stops until the newly selected vehicle is heard from. Tracking then restarts from that vehicle's position, rather than being smoothed across from the old one.

The type of position message in use is also reset by the change, so if the previous lead vehicle was sending ``FOLLOW_TARGET``, ``GLOBAL_POSITION_INT`` from the new lead vehicle is accepted immediately.

Setting :ref:`FOLL_SYSID<FOLL_SYSID>` to "0" deselects the lead vehicle altogether, and nothing is followed until a non-zero system id is set. Earlier firmware treated "0" as "follow the first vehicle seen" and wrote that vehicle's system id into the parameter; it no longer does either, so the lead vehicle must always be selected explicitly.

.. image:: ../images/FollowMode.jpg
   :target: ../_images/FollowMode.jpg

.. image:: ../images/FollowMode-Relative.jpg
   :target: ../_images/FollowMode-Relative.jpg
