.. _common-powermodule-landingpage:

=============================================
Battery Monitors (aka Power Monitors/Modules)
=============================================

.. image:: ../../../images/PowerModule_landingpage.jpg

The links below have information about the most commonly used power monitors/modules

Up to 10 batteries/power monitors can be monitored. Each monitor has its own group of configuration parameters, designated by ``BATTx_`` with x denoting each monitor in the system (first monitor "x" is null character, ie ``BATT_`` prefix). In addition, a ``BATT_MONITOR`` "type" = SUM is available, that consolidates all following (higher numbered) battery monitors into a single report.

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

CAN/DroneCAN Power Monitor
==========================

.. toctree::
    :maxdepth: 1

    CUAV CAN/DroneCAN PMU<common-can-pmu>
    Matek CAN-L4-BM DroneCan PMU <http://www.mateksys.com/?portfolio=can-l4-bm>
    Packet Digital MPPT Solar Controller <common-packetdigital-mppt>
    Pomegranate Systems<common-pomegranate-systems-pm>

I2C Power Monitor
=================

.. toctree::
    :maxdepth: 1

    Rotoye BatMon Smart Battery <common-smart-battery-rotoye.rst>

Power Monitoring Via Telemetry Equipped BLHeli32/S ESCs
=======================================================

- See :ref:`this section<esc-telemetry-based-battery-monitor>` of the :ref:`blheli32-esc-telemetry` page

Liquid Fuel Monitors
====================

.. toctree::
    :maxdepth: 1

    Fuel Monitors <common-fuel-sensors>

Power Monitors/Modules provide these benefits:

- Most provide a stable power supply to the autopilot, in addition to monitoring, and therefore reduces the chance of a brown-out
- Allows real-time monitoring of the battery’s voltage and current and triggering a low battery failsafe
- Allows compensating for the interference on the compass from the motors using the COMPASS_MOT_x and COMPASS_PMOT_x parameters, see :ref:`common-compass-setup-advanced`.
