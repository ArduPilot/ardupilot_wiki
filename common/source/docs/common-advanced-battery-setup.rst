.. _common-advanced-battery-setup:

=============================================
Advanced Advanced Power Monitor Configuration
=============================================

In addition to the normal battery current and voltage setup and scaling parameters addressed in each individual :ref:`power monitor's<common-powermodule-landingpage>` setup page, there are several other parameters available for advanced configuration. Each parameter is well documented by following its link below:

.. note:: While links are provided for the first battery monitor parameter group, there can be up to 10 total monitors in the system depending on the individual autopilot's capabilities and/or use of SMBUS or UAVCAN monitors. These are accessed using ``BATTx_`` parameter prefixes

Parameters for advanced power monitor failsafe setup
----------------------------------------------------

:ref:`BATT_WATT_MAX<BATT_WATT_MAX>`

:ref:`BATT_LOW_TIMER<BATT_LOW_TIMER>`

:ref:`BATT_FS_VOLTSRC<BATT_FS_VOLTSRC>`

:ref:`BATT_LOW_VOLT<BATT_LOW_VOLT>`

:ref:`BATT_LOW_MAH<BATT_LOW_MAH>`

:ref:`BATT_CRT_VOLT<BATT_CRT_VOLT>`

:ref:`BATT_LOW_MAH<BATT_LOW_MAH>`

:ref:`BATT_FS_LOW_ACT<BATT_FS_LOW_ACT>`

:ref:`BATT_FS_CRT_ACT<BATT_FS_CRT_ACT>`

Parameters for preventing arming in the event of a discharged battery
---------------------------------------------------------------------

:ref:`BATT_ARM_VOLT<BATT_ARM_VOLT>`

:ref:`BATT_ARM_MAH<BATT_ARM_MAH>`

