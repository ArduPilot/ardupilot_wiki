=========
Time Sync
=========

.. code:: bash

    module load system_time
    
This module syncs the connected autopilot with the system (GCS) time. This is useful if the autopilot
does not have a GPS module.

It does this by sending ``SYSTEM_TIME`` MAVLink packets to the autopilot.

The time sync will start when the module is loaded and continue until the module is unloaded.

The module has the following settings, which via be set via ``system_time set``.

==================   ===============================================  ===============================
Setting              Description                                      Default
==================   ===============================================  ===============================
verbose              Show debugging and performance stats             False
interval_timesync    How often to send a timesync message (seconds)   10
interval             How often to send the system time (seconds)      10
==================   ===============================================  ===============================

Note: you will need to set bit 1 (MAVLINK_SYSTEM_TIME) in the :ref:`BRD_RTC_TYPES <BRD_RTC_TYPES>` bitmask on the autopilot to enable updates from ``SYSTEM_TIME`` messages.
