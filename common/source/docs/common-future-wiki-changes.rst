.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]

on :ref:`common-auxiliary-functions`, add:
==========================================

+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|       170            |  QSTABILIZE mode           |          |  x      |         |
+----------------------+----------------------------+----------+---------+---------+

on :ref:`common-downloading-and-analyzing-data-logs-in-mission-planner` page:
=============================================================================

under Logging Parameters, change:
- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to one will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures.

to
- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to 1 will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures. Setting to 2 will only log on power application other than USB power to prevent logging while setting up on the bench.

on :ref:`common-powermodule-landingpage`, add:
==============================================

.. toctree::
    :maxdepth: 1

    Synthetic Current Sensor/Analog Voltage Monitor <common-synthetic-current-monitor>



on :ref:`common-uavcan-setup-advanced`, :ref:`mission-planner-initial-setup`, :ref:`common-slcan-f4`, and :ref:`common-slcan-f7h7` pages add the following note:
----------------------------------------------------------------------------------------------------------------------------------------------------------------
.. note:: SLCAN access via COM port is disabled when armed to lower cpu load. Use SLCAN via MAVLink instead.


[site wiki="plane"]

on :ref:`acro-mode` page, in section "Acro Locking", add:
=========================================================

It is recommended that it be set to "2", instead of "1", in order to use a quarternion based control system with much better performance than the older system. In order for this to be effective, yaw rate control (:ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>`) must be "1" and the yaw rate controller tuned using :ref:`Autotune <automatic-tuning-with-autotune>` for best performance.

on the :ref:`automatic-tuning-with-autotune` page:
==================================================

add in the setup section:
-------------------------

The :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>` bitmask selects which axes will be tuned while in Autotune. Default is roll, pitch and yaw.

change note in the YAW Controller section to:
---------------------------------------------

.. note:: while AutoTuning with this controller enabled, roll inputs will result in yaw outputs also, allowing more coordinated turns with the yaw controller active. This will normally result in simultaneously tuning the yaw controller with the roll controller, but not necessisarily completing the yaw tune when the roll tune finishes. Also, there may be seemingly excessive rudder applied initially in the roll tune on vehicles with large yaw authority, until the tune progresses.

[/site]

[site wiki="copter"]

on :ref:`common-transmitter-tuning` page, add:
==============================================

under TUNE parameter table:

+--------+-------------------------+----------------------------------------------------------------------+
|Value	 |Meaning                  | Parameter                                                            |
+========+=========================+======================================================================+
|59      |Position Control Max     |  :ref:`PSC_ANGLE_MAX<PSC_ANGLE_MAX>`                                 |
|        | Lean Angle              |                                                                      |
+--------+-------------------------+----------------------------------------------------------------------+
[/site]