.. _common-efi:
[copywiki destination="plane"]
=========================
Electronic Fuel Injectors
=========================


.. image:: ../../../images/efi.jpg

Photo courtesy of `Lutan Engineering <https://www.lutanefi.com/en/products/uavengine>`__

ArduPilot provides support for several types of electronic fuel injection controllers for :ref:`internal combustion engines<common-ice>`. Both CAN and Serial communication interfaces to the EFI are supported.

Most units provide status information on RPM, Fuel flow, consumption, temperature, etc.

This information is reported in the autopilot logs via the EFI, EFI2, and ECYL log messages. Real time data is sent in the EFI MAVLink messages for monitoring by the GCS.

The following units have been tested and are supported:

- MegaSquirt Serial
- NWPMU CAN bus
- Lutan Serial
- DroneCAN (adapted versions of the above serial units)
- PiccoloCAN (`Currawong's ECU <https://www.currawong.aero/ecu/>`__ and `IntelliJect EFI <https://power4flight.com/uav-engine-products/uav-engine-control-units/intelliject-efi/>`__)

Using :ref:`LUA script drivers <common-lua-scripts>`:

- `DLA EFI (DLA232cc UAV Engine) <https://www.austars-model.com/dla-232cc-uavuas-engine-optional-one-key-startauto-startergenerator_g17937.html>`__
- `HFE CAN EFI driver <https://www.unmannedsystemstechnology.com/company/hfe-international-llc/>`__
- `SkyPower CAN SGC-352 <https://3w-international.com/produkt/sgc-352-starter-generator-control-unit/>`__
- `INF EFI <https://innoflighttechnology.com/efi/>`__
- `Halo-6000 Generator <https://www.foxtechfpv.com/foxtech-halo-6000-efi-generator-for-hybrid-drone.html>`__

Serial Setup
============

For the example below, the unit will assumed to be attached to SERIAL5 of the autopilot.

- :ref:`EFI_TYPE<EFI_TYPE>`: MegaSquirt = 1, Lutan = 3
- :ref:`SERIAL5_PROTOCOL<SERIAL5_PROTOCOL>` = 24 (Serial EFI))
- :ref:`SERIAL5_BAUD<SERIAL5_BAUD>` = 115 (115.2Kbaud)

NWPMU Setup
===========

For the example below, the unit will be assumed to be attached to the first CAN port.

- :ref:`EFI_TYPE<EFI_TYPE>` = 2 (NWPMU)
- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1 (first driver)
- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 6 (EFI_NWPMU)

DroneCAN Setup
==============

For the example below, the unit will be assumed to be attached to the first CAN port.

- :ref:`EFI_TYPE<EFI_TYPE>` = 5 (DroneCAN)
- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1 (first driver)
- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 1 (DroneCAN)

PiccoloCAN Setup
================

For the example below, the unit will be assumed to be attached to the first CAN port.

- :ref:`EFI_TYPE<EFI_TYPE>` = 6 (Currawong-ECU)
- :ref:`EFI_FUEL_DENS<EFI_FUEL_DENS>`: Fuel density to calculate fuel consumption
- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` = 1 (first driver)
- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` = 4 (PiccoloCAN)
- :ref:`CAN_D1_PC_ECU_ID<CAN_D1_PC_ECU_ID>`: Node ID to send throttle commands (Set to 0 to only receive telemetry)

DLA Setup
=========

Driver is provided via a :ref:`LUA <common-lua-scripts>` script. See LUA script setup instructions and the `DLA EFI Lua md file <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/drivers/EFI_DLA.md>`__ for details.

Setup using LUA script drivers
==============================

- Be sure your autopilot is capable of running LUA scripts. See :ref:`binary-features` listing for your autopilot and be sure it does NOT list "SCRIPTING" under "Other" features as missing.
- Setup :ref:`LUA Scripting <common-lua-scripts>`
- Copy the appropriate driver script to your autopilot SD card from the `LUA Driver directory <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/drivers>`__
- Follow the instructions in the driver's ".md" file.

Using a Battery Monitor to Report Fuel Flow and Consumption
===========================================================

If an EFI is used in the system, either thru a LUA driver or the built-in drivers above, the fuel flow and consumption can be monitored using :ref:`BATT_MONITOR<BATT_MONITOR>` = 27. The fuel flow in liters/hour will be reported as amps, while the fuel consumed in milliliters will be reported as mah.

.. note:: the MAVLink command to reset the fuel consumed does not work with this monitor.

Displaying EFI telemetry in Mission Planner
===========================================

The following EFI telemetry is available in the Mission Planner. Just right click in the DATA screens HUD display and select ``User Items``. Then select what data to display.

.. image:: ../../../images/mp_efi_hud_items.jpg
   :target: ../_images/mp_efi_hud_items.jpg

Using the EFI RPM in ArduPilot
==============================

The EFI's rpm telemetry can be used as the autopilot's RPM sensor, for display in the GCS as above, or for use with the harmonic notch filter.

First, set either RPM1 or RPM2 sensor as being sourced from the EFI (examples will use RPM1):

- set :ref:`RPM1_TYPE<RPM1_TYPE>` = 3 (EFI)
- then setup the :ref:`common-rpm-based-notch`
