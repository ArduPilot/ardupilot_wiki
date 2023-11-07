.. _common-gps-trimble-px1:


=====================
Trimble PX-1 RTX
=====================

The Trimble PX-1 RTX is a relatively expensive but also highly accurate RTX GNSS+INS.
It supports corrections from a satellite or cellular network through `CenterPoint® RTX Correction service <https://positioningservices.trimble.com/en/rtx>`__.
RTX provides centimeter accuracy without the use of a base station.
Although the PX-1 is designed to be used as an external inertial navigation system, it is also supported for use as a strict GPS in ArduPilot.
Detailed `information from the manufacturer can be found here <https://advancedairmobility.trimble.com/>`__.

Parameter Setup
----------------------

The following instructions assume the PX-1 is your first GPS. If you have configured it as a second GPS, change all parameter names.

- :ref:`GPS_COM_PORT <GPS_COM_PORT>` = 1 (to use COM2, the TTL output of the PX-1) or 3 (to use COM1, the RS232 output of the PX-1)
- :ref:`GPS_TYPE <GPS_TYPE>` = 11 (for GSOF)
- :ref:`SERIAL3_PROTOCOL <SERIAL3_PROTOCOL>` = 5 (to use GPS)

The following parameters are not yet supported by GSOF:
- :ref:`GPS_RATE_MS<GPS_RATE_MS>`
- :ref:`GPS_AUTO_CONFIG<GPS_AUTO_CONFIG>`
- :ref:`GPS_SAVE_CFG<GPS_SAVE_CFG>`
- :ref:`GPS_GNSS_MODE<GPS_GNSS_MODE>`
- :ref:`GPS_RAW_DATA<GPS_RAW_DATA>`
- :ref:`GPS_NAVFILTER<GPS_NAVFILTER>`
