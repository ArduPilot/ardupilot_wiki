.. _mavlink-calibration:

==================
Sensor Calibration
==================

This page explains how MAVLink can be used by a ground station or companion computer to trigger sensor calibrations. The user wiki pages for these procedures are here: :ref:`accelerometer <copter:common-accelerometer-calibration>`, :ref:`compass <copter:common-compass-calibration-in-mission-planner>`, :ref:`radio <copter:common-radio-control-calibration>`.

.. note::

   The vehicle must be disarmed before starting a calibration. If
   armed, ``MAV_CMD_PREFLIGHT_CALIBRATION`` replies with a "Disarm to
   allow calibration" status text and is rejected, and
   ``MAV_CMD_DO_START_MAG_CAL`` replies with "Disarm to allow compass
   calibration". ``MAV_CMD_DO_ACCEPT_MAG_CAL`` and
   ``MAV_CMD_DO_CANCEL_MAG_CAL`` have no armed-state check.

Simple calibrations
--------------------

Gyro, barometer, board-level (trim) and simple accelerometer calibrations are each triggered with a single `MAV_CMD_PREFLIGHT_CALIBRATION <https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION>`__ sent within a `COMMAND_INT <https://mavlink.io/en/messages/common.html#COMMAND_INT>`__ (or ``COMMAND_LONG``). Only one of the fields below should be non-zero in a given message.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Command Field</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>param1</strong></td>
   <td>float</td>
   <td>1: gyro calibration</td>
   </tr>
   <tr>
   <td><strong>param2</strong></td>
   <td>float</td>
   <td>magnetometer calibration — see below</td>
   </tr>
   <tr>
   <td><strong>param3</strong></td>
   <td>float</td>
   <td>1: barometer (and airspeed, if fitted) calibration</td>
   </tr>
   <tr>
   <td>param4</td>
   <td>float</td>
   <td>positive value: sets an internal "RC calibrating" flag that blocks arming (arming check fails with "RC calibrating") while true; zero/negative clears it — but only when param1 and param3 are both 0. ArduPilot does not itself run an RC endpoint calibration routine in response to this field. See note below</td>
   </tr>
   <tr>
   <td><strong>x</strong> (param5)</td>
   <td>int32_t</td>
   <td>Accelerometer calibration action:

   1: full 6-position calibration (see below)

   2: board-level/trim calibration

   4: simple (single position) accelerometer calibration

   76: force-accept the existing accelerometer calibration as valid, without re-running it
   </td>
   </tr>
   <tr>
   <td>y (param6)</td>
   <td>int32_t</td>
   <td>Vehicle-specific: Copter — 1: CompassMot calibration. Rover — 1: windvane direction calibration, 2: windvane speed calibration. Sub — 1: rejected (MAV_RESULT_UNSUPPORTED), CompassMot is not supported. Plane, AntennaTracker and Blimp: not used.</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>z (param7)</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   </tbody>
   </table>

.. note::

   ``x``/``y`` are ``int32_t`` fields of ``COMMAND_INT``, not ``float``. When
   this command is sent as ``COMMAND_LONG`` instead, its float param5/param6
   are cast directly to ``x``/``y`` on receipt.

.. note::

   ArduPilot does not itself run an RC endpoint calibration routine in
   response to param4 — see the :ref:`radio calibration page
   <copter:common-radio-control-calibration>` for how RC calibration
   actually works. The "RC calibrating" flag it does set/clear is
   only actually evaluated when param1 and param3 are both 0 — a
   command that also triggers a gyro (param1=1) or barometer
   (param3=1) calibration returns before reaching the param4 check,
   so param4 has no effect in that same message. In practice this
   means: combining a gyro or barometer calibration with param4=0
   will **not** clear a flag left set by an earlier command — a
   separate command with param1=0, param3=0 and param4 ≤ 0 (or a
   reboot) is required to clear it. Conversely, an unrelated
   accelerometer-calibration command (param5/``x`` set, param1=0,
   param3=0) with param4 left at its default of 0 **will** clear the
   flag, even in the middle of an in-progress RC calibration.

ArduPilot only implements one action for the magnetometer's param2
field: a value of 76 force-accepts the existing compass calibration as
valid without re-running it (useful after a parameter reload that
cleared the calibration-valid flags). The MAVLink spec also defines a
value of 1 to *start* a magnetometer calibration via this same
command, but ArduPilot does not implement that path — sending param2 =
1 alone returns ``MAV_RESULT_UNSUPPORTED``. Use the dedicated
:ref:`compass calibration commands <mavlink-calibration_compass-calibration>` below to actually start a mag calibration.

Full 6-position accelerometer calibration
------------------------------------------

Sending ``MAV_CMD_PREFLIGHT_CALIBRATION`` with x (param5) = 1 starts
the interactive full accelerometer calibration, which requires the
vehicle to be placed in each of 6 orientations in turn. ArduPilot
drives this using `MAV_CMD_ACCELCAL_VEHICLE_POS <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_ACCELCAL_VEHICLE_POS>`__:

.. note::

   Before starting the position wizard, ArduPilot first runs a gyro
   calibration internally. If that gyro calibration fails (for
   example the vehicle is being handled or vibrating), the whole
   command fails immediately with ``MAV_RESULT_FAILED`` and no
   ``MAV_CMD_ACCELCAL_VEHICLE_POS`` is ever sent — a GCS that has
   already shown a "place vehicle LEVEL" prompt and is waiting for
   that message will wait forever unless it also handles this failure
   result.

- ArduPilot sends this command *to the GCS* with param1 set to the position it wants next (`ACCELCAL_VEHICLE_POS <https://mavlink.io/en/messages/ardupilotmega.html#ACCELCAL_VEHICLE_POS>`__: 1=LEVEL, 2=LEFT, 3=RIGHT, 4=NOSEDOWN, 5=NOSEUP, 6=BACK), which the GCS should show to the user
- once the vehicle has been placed in that position (and is stationary), the GCS sends the same command *back to the vehicle* with the same param1 value to confirm that position is ready and trigger capture of that orientation's data
- ArduPilot then either requests the next position, or ends the calibration by sending ``MAV_CMD_ACCELCAL_VEHICLE_POS`` with param1 = 16777215 (success) or 16777216 (failed)

.. _mavlink-calibration_compass-calibration:

Compass (magnetometer) calibration
-------------------------------------

Compass calibration uses three dedicated commands rather than
``MAV_CMD_PREFLIGHT_CALIBRATION``. Each is sent within a
``COMMAND_INT``/``COMMAND_LONG`` with a bitmask of which compasses to
act on in param1 (0 means all):

- `MAV_CMD_DO_START_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL>`__ — start calibration. param2 = retry on failure (0/1), param3 = autosave without waiting for ``MAV_CMD_DO_ACCEPT_MAG_CAL`` (0/1), param4 = delay in seconds before starting, x (param5) = autoreboot when done (0/1)
- `MAV_CMD_DO_ACCEPT_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_ACCEPT_MAG_CAL>`__ — accept/save a completed calibration (only needed if autosave was not requested)
- `MAV_CMD_DO_CANCEL_MAG_CAL <https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_CANCEL_MAG_CAL>`__ — cancel a running calibration

While a compass calibration is running, ArduPilot streams `MAG_CAL_PROGRESS <https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS>`__ messages showing each compass's ``completion_pct``. Both ``MAG_CAL_PROGRESS`` and ``MAG_CAL_REPORT`` (below) are sent as part of the ``EXTRA3`` telemetry stream, so a GCS/companion computer that has that stream's rate set to 0 will not see them, and calibration will appear to hang even though it is progressing normally.

.. note::

   ``MAG_CAL_REPORT`` is not a one-shot message. Once a compass finishes calibrating, ArduPilot has no "already reported" latch — it keeps re-sending that compass's ``MAG_CAL_REPORT`` at the ``EXTRA3`` stream rate rather than sending it just once. Its ``cal_status`` field gives the fit result (success/failed), and a separate ``autosaved`` field indicates whether the calibration has actually been saved to the compass's offset parameters (true if autosave was requested via ``MAV_CMD_DO_START_MAG_CAL``'s param3, or after ``MAV_CMD_DO_ACCEPT_MAG_CAL`` is sent) — the two are independent, so a compass can report a successful fit that has not yet been saved. A client that treats the first ``MAG_CAL_REPORT`` as a single completion event should still expect further copies of it to keep arriving.

**Example**

The example commands below can be copy-pasted into MAVProxy (aka SITL) to test these commands. Before running these commands enter, "module load message"

- ``message COMMAND_LONG 0 0 241 0 1 0 0 0 0 0 0`` — gyro calibration
- ``message COMMAND_LONG 0 0 241 0 0 0 1 0 0 0 0`` — barometer calibration
- ``message COMMAND_LONG 0 0 241 0 0 0 0 0 4 0 0`` — simple accelerometer calibration
- ``message COMMAND_LONG 0 0 42424 0 0 0 1 0 0 0 0`` — start compass calibration (autosave)
