.. _common-ekf-sources:

==================================
EKF Source Selection and Switching
==================================

The EKF position and velocity sources used for its internal updates can be set using the ``EKx_SRCn_y`` parameters. For EKF3's primary sources, these are:

- :ref:`EK3_SRC1_POSXY<EK3_SRC1_POSXY>`: Position Horizontal Source
- :ref:`EK3_SRC1_VELXY<EK3_SRC1_VELXY>`: Velocity Horizontal Source
- :ref:`EK3_SRC1_POSZ<EK3_SRC1_POSZ>`: Position Vertical Source
- :ref:`EK3_SRC1_VELZ<EK3_SRC1_VELZ>`: Velocity Vertical Source
- :ref:`EK3_SRC1_YAW<EK3_SRC1_YAW>`: Yaw Source

The options for these are generally self-explanatory however a couple of items are explained in more detail below

The :ref:`EK3_SRC1_POSZ<EK3_SRC1_POSZ>` source options are:

- Option 1: Baro is the default and works well for most vehicles and situations.  If a GPS is also present the EKF may be configured to slowly correct the altitude to match the GPS by setting :ref:`EK3_OGN_HGT_MASK <EK3_OGN_HGT_MASK>` to 5 (e.g. 1:Correct when using Baro height + 4:Apply corrections to local position).
- Option 2: RangeFinder should almost never be used.  This is only appropriate for indoor use where the floor is flat with no ground clutter (e.g. no chairs, boxes, etc).  Please note that :ref:`Surface Tracking <copter:terrain-following-manual-modes>` and :ref:`Terrain Following <terrain-following>` do not require the EKF to use the rangefinder at all
- Option 3: GPS is only recommended if the vehicle will fly long duration flights where the air pressure may change significantly **and** the vehicle has a high quality GPS (e.g. UBlox F9P dual band).  In case of GPS failure the EKF will fallback to using the barometer (if present)
- Option 4: Beacon may be useful when beacons are used in place of a GPS
- Option 6: ExternalNav may be used when a companion device provides a position estimate

The :ref:`EK3_SRC1_YAW<EK3_SRC1_YAW>` source options are:

- Option 1: Compass is the normal default.
- Option 2: GPS is used with GPS that can supply yaw (see :ref:`common-gps-for-yaw`)
- Option 3: GPS with compass fallback is the same as GPS option, but while it has GPS yaw available it learns compass offsets to make the compass yaw match the GPS yaw. This means that if you lose GPS yaw while flying then the compass is ready to takeover immediately, with a smooth change from GPS yaw to compass yaw.
- Option 6: ExternalNav is used when a companion device is providing yaw information
- Option 8: GSF (Gaussian Sum Filter) provides yaw information based on GPS velocity and position changes and Plane will automatically use this in :ref:`compass-less <common-compassless>` configurations,even if not selected. Copter requires this to be set explicitly in these configurations.

Source Switching
================

Three sets of EKF position and velocity source parameters are provided and by default the 1st set is used (e.g. the _SRC1 set).

The active set can be selected via an :ref:`RC auxiliary switch <common-auxiliary-functions>` (e.g. set ``RCx_OPTION`` to "90" / "EKF Position Source").

Ground stations or companion computers may set the source by sending a `MAV_CMD_SET_EKF_SOURCE_SET  <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_SET_EKF_SOURCE_SET>`__ mavlink command but no GCSs are currently known to implement this.

This feature is especially helpful when using :ref:`common-non-gps-to-gps`.

Velocity Source Fusing
======================

There is an option to fuse all sources of velocity into the EKF.  This will take velocity information from the XY and Z sources defined by :``EK3_SRCx_VELXY`` and ``EK3_SRCx_VELZ``, where 'x' is the source number (1-3).  To enable this option set the first bit in the :ref:`EK3_SRC_OPTIONS<EK3_SRC_OPTIONS>` bitmask.

.. warning:: When using this option, users should ensure that the velocity measurements being provided to the EKF are both in the same reference frame/co-ordinate system.

