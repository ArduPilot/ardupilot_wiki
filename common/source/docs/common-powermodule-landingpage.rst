.. _common-powermodule-landingpage:

=============================================
Battery Monitors (aka Power Monitors/Modules)
=============================================

.. image:: ../../../images/PowerModule_landingpage.jpg

Power Monitors/Modules provide these benefits:

- Most provide a stable power supply to the autopilot, in addition to monitoring, and therefore reduces the chance of a brown-out
- Allows real-time monitoring of the battery’s voltage and current and triggering a low battery failsafe
- Allows compensating for the interference on the compass from the motors using the COMPASS_MOT_x and COMPASS_PMOT_x parameters, see :ref:`common-compass-setup-advanced`.

The links below have information about the most commonly used power monitors/modules

Up to 16 batteries/power monitors can be monitored. Each monitor has its own group of configuration parameters, designated by ``BATTx_`` with x denoting each monitor in the system (first monitor "x" is null character, ie ``BATT_`` prefix). In addition, a ``BATT_MONITOR`` "type" = SUM is available, that consolidates  battery monitors into a single report using the :ref:`BATT_SUM_MASK<BATT_SUM_MASK>`.

.. note:: some kinds of monitors can provide bi-directional battery current information. These are useful when generators or MPPT chargers are being utilized in the system to monitor the net charge state of the battery.

.. toctree::
    :maxdepth: 1

    Power Monitor Configuration <common-power-module-configuration-in-mission-planner>

Power Monitors Connecting to AutoPilot Power Monitor Port
=========================================================

.. toctree::
    :maxdepth: 1
    
    Common Power Module <common-3dr-power-module>
    AirbotPower Power Module <common-airbotpower-power-module>
    CUAV HV PM<common-hv-pm>
    Mauch Power Monitor <common-mauch-power-modules>
    SmartAP Power Distribution Board<common-smartap-pdb>
    Synthetic Current Sensor/Analog Voltage Monitor <common-synthetic-current-monitor>

CAN/DroneCAN Power Monitors and Batteries
=========================================

.. toctree::
    :maxdepth: 1

    CUAV CAN/DroneCAN PMU<common-can-pmu>
    Matek CAN-L4-BM DroneCan PMU <http://www.mateksys.com/?portfolio=can-l4-bm>
    Packet Digital MPPT Solar Controller <common-packetdigital-mppt>
    Pomegranate Systems<common-pomegranate-systems-pm>
    Tattu DroneCan Battery<common-tattu-dronecan-battery>
    Aerotate DroneCAN Smart Battery<common-aerotate-dronecan-battery>

I2C Power Monitor
=================

.. toctree::
    :maxdepth: 1

    Rotoye BatMon Smart Battery <common-smart-battery-rotoye.rst>

Power Monitoring Via Telemetry Equipped BLHeli32/S ESCs
=======================================================

- See :ref:`this section<esc-telemetry-based-battery-monitor>` of the :ref:`blheli32-esc-telemetry` page


EFI Fuel Monitoring
===================

EFI Fuel Monitoring, See :ref:`common-efi` section on using :ref:`BATT_MONITOR<BATT_MONITOR>` = 27 (EFI).

Liquid Fuel Monitors
====================

.. toctree::
    :maxdepth: 1

    Fuel Monitors <common-fuel-sensors>


Substituting  a Battery Monitor's Data into an ESC's telemetry stream
=====================================================================

- See :ref:`BATT_ESC_INDEX<BATT_ESC_INDEX>` (for first Battery Monitor. ``BATTx_ESC_INDEX`` for others.)

.. note:: this feature is usually only available as a build option using the `Custom Firmware Build Server <https://custom.ardupilot.org>`__

