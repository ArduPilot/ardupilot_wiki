.. _common-marvelmind:

=================================
Marvelmind for Non-GPS navigation
=================================

[copywiki destination="copter,rover,blimp"]

This article explains how a `MarvelMind <https://marvelmind.com/>`__ system can be a short-range substitute for a GPS allowing position control modes like Auto and Guided.
See the `user manual <https://marvelmind.com/pics/marvelmind_navigation_system_manual.pdf>`__ for more details on the system.

.. image:: ../../../images/marvel-mind.jpg
    :target: ../_images/marvel-mind.jpg

*image courtesy of marvelmind.com*

Required Hardware
=================

* `Marvelmind Starter Set <https://marvelmind.com/product/starter-set-hw-v4-9-plastic-housing/>`__

Marvelmind Settings
=====================

- Ensure the Marvelmind is firmware version 5.77 (or higher) and enable the 'Raw Distances Data' in the Dashboard
- Set the stationary beacon with the lowest address as the origin in your dashboard.
- The height of every stationary beacon, from the ground, must be set on the dashboard.
- The hedge should be set to output Marvelmind Protocol at baud rate of 115200.

Connecting to an autopilot
==========================

- Set :ref:`EK3_SRC1_POSXY<EK3_SRC1_POSXY>` to 4 (Beacon)
- Set :ref:`EK3_SRC1_VELXY<EK3_SRC1_VELXY>` to 0 (None)
- Set :ref:`EK3_SRC1_POSZ<EK3_SRC1_POSZ>` to 4 (Beacon)
- Set :ref:`EK3_SRC1_VELZ<EK3_SRC1_VELZ>` to 0 (None)
- Set :ref:`EK3_SRC1_YAW<EK3_SRC1_YAW>` to 1 (Compass)
- Set :ref:`BCN_TYPE <BCN_TYPE>` to 2 (means using Marvelmind system)
- Set :ref:`BCN_LATITUDE <BCN_LATITUDE>`, :ref:`BCN_LONGITUDE <BCN_LONGITUDE>` and :ref:`BCN_ALT <BCN_ALT>` to match your actual location.  Getting these values exactly correct is not particularly important although getting it close is required in order for the compass's declination to be correctly looked up from the small database held within ArduPilot.
- Set :ref:`BCN_ORIENT_YAW <BCN_ORIENT_YAW>` to the heading from the origin beacon to the 2nd beacon.  One way to capture this value is to stand at the origin holding the vehicle so that it's nose points towards the second beacon.  Read the vehicle's heading from the HUD and enter this value into :ref:`BCN_ORIENT_YAW <BCN_ORIENT_YAW>`
- Set :ref:`GPS1_TYPE <GPS1_TYPE>` to 0 to disable the GPS
- Set :ref:`ARMING_CHECK <ARMING_CHECK>` to -9 to disable the GPS arming check
- Set :ref:`SERIAL1_BAUD <SERIAL1_BAUD>` to 115 to set telemetry1's baud rate to 115200 (if using Telem2 set :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` instead)
- Set :ref:`SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` to 13 to enable receiving data from the marvelmind (If using Telem2 set :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` instead)
- Set :ref:`BRD_SER1_RTSCTS <BRD_SER1_RTSCTS>` to 0 to ensure telem1 does not use flow control (If using Telem2 set :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` instead, not required if using Serial4/5)

Ground Testing
==============

- Connect the Autopilot to a ground station
- Click on 'freeze map' on the dashboard
- After about 1 minute the vehicle's position should jump to the latitude, longitude you input during the configuration step (above)
- Check that the vehicle's position is relatively stable (i.e. moving around less than one meter)
- Walk the vehicle around between the beacons and ensure that its position on the map updates correctly

DataFlash logging
=================

The distance to the beacons can be found in the dataflash log's BCN message's D0, D1, D2, D3 fields.
