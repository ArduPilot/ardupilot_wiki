.. _mavlink-mission-upload-download:

=========================
Mission Upload / Download
=========================

This page explains how MAVLink can be used to upload and download missions (executed in :ref:`Auto mode <copter:auto-mode>`) and perform some other mission related actions.  The user wiki page for :ref:`Mission Planning is here <copter:common-mission-planning>`.

Mission Related Messages
------------------------

- `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`__
- `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`__
- `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__
- `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__
- `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`__
- `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`__
- `MISSION_REQUEST_INT <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT>`__
- `MISSION_REQUEST_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST>`__
- `MISSION_REQUEST_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST_PARTIAL_LIST>`__
- `MISSION_SET_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT>`__
- `MISSION_WRITE_PARTIAL_LIST <https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST>`__

Also the following MAV_CMDs may be sent within a COMMAND_LONG

- `MAV_CMD_MISSION_START <https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START>`__
- `MAV_CMD_DO_CHANGE_SPEED <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED>`__
- `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`__

Pausing a Mission with MAV_CMD_DO_PAUSE_CONTINUE
------------------------------------------------

A mission can be paused or resumed by sending a `COMMAND_LONG <https://mavlink.io/en/messages/common.html#COMMAND_LONG>`__ with the command, param1 and param2 fields set as specified for the `MAV_CMD_DO_PAUSE_CONTINUE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE>`__ command.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>target_system</strong></td>
   <td>uint8_t</td>
   <td>System ID</td>
   </tr>
   <tr>
   <td><strong>target_component</strong></td>
   <td>uint8_t</td>
   <td>Component ID of flight controller or just 0</td>
   </tr>
   <tr>
   <td><strong>command</strong></td>
   <td>uint16_t</td>
   <td>MAV_CMD_DO_PAUSE_CONTINUE=193</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>confirmation</strong></td>
   <td>uint8_t</td>
   <td>0</td>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>0:pause, 1:continue</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param4</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param5</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param6</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td><strong>param7</strong></td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test this command.  Before running these commands enter, "module load message"

+------------------------------------------------------+---------------------------+
| Example MAVProxy/SITL Command                        | Description               |
+======================================================+===========================+
| ``message COMMAND_LONG 0 0 193 0 0 0 0 0 0 0 0``     | pause mission             |
+------------------------------------------------------+---------------------------+
| ``message COMMAND_LONG 0 0 193 0 1 0 0 0 0 0 0``     | continue / resume mission |
+------------------------------------------------------+---------------------------+

Fence and Rally Point Upload/Download
-------------------------------------

Geofences and rally points use exactly the same message protocol
described above (``MISSION_COUNT``, ``MISSION_ITEM_INT``,
``MISSION_REQUEST_INT``, ``MISSION_ACK``, etc). The only difference is
that the ``mission_type`` field of each message is set to
`MAV_MISSION_TYPE_FENCE (1) <https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_FENCE>`__
or `MAV_MISSION_TYPE_RALLY (2) <https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_RALLY>`__
instead of the default ``MAV_MISSION_TYPE_MISSION (0)``. Fences and
rally points are uploaded, downloaded and cleared independently of the
main mission and of each other.

.. note::

   ``mission_type`` is a MAVLink 2 extension field. ArduPilot rejects
   fence/rally *upload or download* (``MISSION_COUNT``,
   ``MISSION_REQUEST_LIST``, ``MISSION_REQUEST_INT``,
   ``MISSION_WRITE_PARTIAL_LIST``, etc) attempted over a MAVLink 1
   connection with a "Need mavlink2 for item transfer" status text;
   only the main mission (``MAV_MISSION_TYPE_MISSION``) works for
   those over MAVLink 1.

.. warning::

   ``MISSION_CLEAR_ALL`` is **not** covered by that MAVLink 2 check.
   A MAVLink 1 client sending ``MISSION_CLEAR_ALL`` with
   ``mission_type`` set to ``MAV_MISSION_TYPE_FENCE`` or
   ``MAV_MISSION_TYPE_RALLY`` will silently erase the stored geofence
   or rally points.

The user wiki pages for :ref:`Fence Setup <copter:common-geofencing-landing-page>` and :ref:`Rally Points <copter:common-rally-points>` explain these features from a GCS-user's perspective.

Fence items
~~~~~~~~~~~

Each fence ``MISSION_ITEM_INT`` commonly uses one of the following
commands:

- `MAV_CMD_NAV_FENCE_RETURN_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_RETURN_POINT>`__ — Plane only (there can be only one): destination used after a fence breach when :ref:`FENCE_ACTION <plane:FENCE_ACTION>` is ``GUIDED`` or ``GUIDED_THROTTLE_PASS`` and :ref:`FENCE_RET_RALLY <plane:FENCE_RET_RALLY>` is 0; if ``FENCE_RET_RALLY`` is 1, Plane instead heads to the nearest rally point or home. Not used for the RTL/Autoland fence actions, and not used at all on Copter
- `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION>`__ — one vertex of an inclusion polygon (vehicle must stay inside); at least 3 required, each with param1 set to the total vertex count of that polygon
- `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION>`__ — one vertex of an exclusion polygon (vehicle must stay outside)
- `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION>`__ — circular area the vehicle must stay inside (param1 = radius in metres)
- `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION>`__ — circular area the vehicle must stay outside (param1 = radius in metres)

.. note::

   All vertices of one polygon must be uploaded contiguously, with an
   identical param1 (vertex count) value on every item; a differing
   item type or param1 value appearing mid-polygon is rejected with a
   "Received incorrect vertex type" status text, and finishing with
   fewer vertices than param1 promised is rejected with "Unexpected
   vertex count"/"Incorrect item count". Circle and return-point items
   are likewise rejected if they interrupt a polygon's vertex run.

ArduPilot also supports ``MAV_CMD_NAV_FENCE_HOME_CIRCLE_INCLUSION``
(5005) — a circular inclusion fence centered on home rather than a
fixed point; if home moves, the fence moves with it.

.. note::

   The MAVLink spec defines an "inclusion group" field (param2) on the
   polygon/circle-inclusion commands, intended to let the vehicle
   require being inside just one of several independent inclusion
   areas. ArduPilot does not currently read or store this field — all
   inclusion areas are combined into a single group.

   By default, when more than one inclusion area is defined, the
   vehicle must be inside *all* of them at once (their intersection).
   Setting bit 1 of :ref:`FENCE_OPTIONS <copter:FENCE_OPTIONS>`
   switches this to a union instead, so the vehicle only needs to be
   inside at least one inclusion area.

Rally items
~~~~~~~~~~~

Each rally point ``MISSION_ITEM_INT`` uses `MAV_CMD_NAV_RALLY_POINT <https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT>`__, with the location given in the item's x/y/z fields (latitude/longitude in degrees ×1e7, altitude in metres, per the mission item's chosen frame). Multiple rally points may be defined; ArduPilot considers the nearest valid rally point when RTL-ing, subject to the :ref:`RALLY_INCL_HOME <copter:RALLY_INCL_HOME>` and :ref:`RALLY_LIMIT_KM <copter:RALLY_LIMIT_KM>` parameters, and will fall back to home if no rally point qualifies.

**Example**

Using MAVProxy/SITL, which understands the ``mission_type`` field:

- ``fence list`` — download and list the fence
- ``rally list`` — download and list rally points
