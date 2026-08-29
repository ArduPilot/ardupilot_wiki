.. _plane-BVLOS-flying:

BVLOS Flight Planning
=====================

This page outlines best practices for Beyond Visual Line of Sight (BVLOS) flight operations, especially for QuadPlanes, although many of the principles also apply to other vehicle types.

Key topics include:

- Types of failsafes
- Recommended failsafe behaviors
- Altitude and terrain handling
- Preplanned return paths
- Use of rally points and contingency landings
- Testing and simulation of failsafe behavior
- Matching SITL models to real aircraft performance

Failsafe Types
--------------

BVLOS operations require careful configuration of multiple failsafe types:

- **GCS Failsafe**: Loss of MAVLink heartbeat or command from the GCS
- **RC Failsafe**: Loss of stick input from the remote controller
- **Propulsion Failsafe**: Loss of forward motor or VTOL lift motors
- **Battery Failsafe**: Low voltage or capacity conditions
- **Sensor Failsafe**: Loss of key sensors or actuators (e.g. GPS, barometer)

Failsafe Behavior
-----------------

Each failsafe condition should have a clearly defined and tested behavior.

GCS Failsafe
^^^^^^^^^^^^

To trigger GCS failsafe on MAVLink communication loss, set:

``FS_GCS_ENABL = 2``
``FS_LONG_ACTN = 1``

These settings cause the vehicle to enter a failsafe after ``FS_LONG_TIMEOUT`` seconds without a valid GCS heartbeat from :ref:`SYSID_MYGCS<SYSID_MYGCS>`.

Only heartbeats from MAVLink system IDs matching :ref:`SYSID_MYGCS<SYSID_MYGCS>` are valid. Confirm that other onboard MAVLink devices do not use the same ID.

Interaction with QRTL
^^^^^^^^^^^^^^^^^^^^^

QuadPlanes may automatically enter QRTL mode upon an RTL event if :ref:`Q_RTL_MODE<Q_RTL_MODE>` is set appropriately. Ensure:

- :ref:`Q_RTL_ALT<Q_RTL_ALT>` is correct
- Terrain following behavior is enabled as needed
- Fixed-wing QRTL entry is enabled if applicable

Testing Failsafe Behavior
-------------------------

Before BVLOS flights, test all failsafe logic under VLOS conditions.

Temporarily change :ref:`SYSID_MYGCS<SYSID_MYGCS>` (e.g. from 255 to 250) during flight to simulate GCS loss.

Recommended setup:

- Circuit mission with :ref:`DO_JUMP<DO_JUMP>` loops lasting at least 30 minutes
- Keep flight within VLOS (<700m)
- Perform SITL tests with matching firmware/parameters before real flights

Check:

- Correct mode transitions (e.g. RTL, QRTL, AUTO)
- Altitude correctness
- Landing sequence activation

Altitude Handling
-----------------

Choose one of two strategies:

- **Terrain-following** with AGL waypoints
- **AMSL** altitude with detailed terrain planning

If using terrain following:

- Preload terrain data from https://terrain.ardupilot.org
- Set: :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>`, :ref:`TERRAIN_LOOKAHD<TERRAIN_LOOKAHD>`, :ref:`TERRAIN_SPACING<TERRAIN_SPACING>`
- Test failsafe altitudes: :ref:`RTL_ALT<RTR_ALT>`, :ref:`Q_RTL_ALT<Q_RTL_ALT>`

Planned Return Paths
--------------------

Use :ref:`DO_LAND_START<DO_LAND_START>` mission items to define planned return/landing sequences.

Set:

:ref:`RTL_AUTOLAND<RTL_AUTOLAND>` = 2

This allows RTL to enter AUTO mode at the nearest :ref:`DO_LAND_START<DO_LAND_START>` mission item.

Contingency Landing Points
--------------------------

You should define alternate landing points for emergencies using:

- Additional :ref:`DO_LAND_START<DO_LAND_START>` items
- Rally points (see below)
- Designated loiter or hold points

Use of Rally Points
-------------------

Rally Points offer predefined emergency landing zones. Configure them through the GCS interface and verify expected behavior during RTL, FS_LONG, or RC failsafe events.

Simulating GCS and RC Failsafe
------------------------------

Use SITL to test RC failsafe by disabling the RC input.

GCS failsafe can be tested by changing :ref:`SYSID_MYGCS<SYSID_MYGCS>` or forcibly stopping MAVLink communication.

Ensure the system transitions to appropriate modes and behaves consistently across these scenarios.

C2 Link Planning
----------------

BVLOS operations require robust C2 (Command and Control) links. Plan for:

- Dual links (e.g., RF and LTE)
- Link monitoring via :ref:`STAT_RESET<STAT_RESET>` and :ref:`LINK_LOSS_ACTION<LINK_LOSS_ACTION>`
- Switching logic or mission logic to handle loss gracefully

Matching SITL to Your Vehicle
-----------------------------

For effective simulation match these parameters:

- :ref:`NAVL1_PERIOD<NAVL1_PERIOD>`
- :ref:`ROLL_LIMIT_DEG<ROLL_LIMIT_DEG>`, :ref:`PTCH_LIM_MIN_DEG<PTCH_LIM_MIN_DEG>`, :ref:`PTCH_LIM_MAX_DEG<PTCH_LIM_MAX_DEG>`
- :ref:`TECS_CLMB_MAX<TECS_CLMB_MAX>`, :ref:`TECS_SINK_MIN<TECS_SINK_MIN>`, :ref:`TECS_PITCH_MAX<TECS_PITCH_MAX>`, :ref:`TECS_PITCH_MIN<TECS_PITCH_MIN>`
- :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>`, :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`, :ref:`AIRSPEED_MAX<AIRSPEED_MAX>`, :ref:`TECS_LAND_ARSPD<TECS_LAND_ARSPD>`
- :ref:`Q_OPTIONS<Q_OPTIONS>`, :ref:`Q_GUIDED_MODE<Q_GUIDED_MODE>`
- :ref:`RTL_AUTOLAND<RTL_AUTOLAND>`, :ref:`WP_LOITER_RAD<WP_LOITER_RAD>`
- :ref:`LEVEL_ROLL_LIMIT<LEVEL_ROLL_LIMIT>`
- All relevant :ref:`FS_*<FS_GCS_ENABL>` parameters
- Any Lua scripts that may influence mission behavior

Test your mission in SITL before deployment.

Further Reading
---------------

- :ref:`plane-failsafe`
- :ref:`terrain-following`
- :ref:`rally-points`
- :ref:`do-land-start`
- :ref:`plane-sitl`
