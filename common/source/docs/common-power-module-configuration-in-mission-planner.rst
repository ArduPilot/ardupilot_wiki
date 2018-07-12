.. _common-power-module-configuration-in-mission-planner:

=============================================
Power Module Configuration in Mission Planner
=============================================

This page explains how to configure and calibrate a Power Module (PM) to
measure battery voltage and current consumption.

Overview
========

A power module can be used to provide a stable power supply to the
system, and to accurately measure the battery voltage/current in order
to trigger a return to launch on low battery. ArduPilot is 
:ref:`compatible with a number of power modules <common-powermodule-landingpage>`.

This article explains how to set up and configure power modules using
*Mission Planner*.

Mission Planner Setup
=====================

Battery measurement is primarily set up in the *Mission Planner*'s
**INITIAL SETUP \| Optional Hardware \| Battery Monitor** screen.

.. figure:: ../../../images/MissionPlanner_BatteryMonitorConfiguration.png
   :target: ../_images/MissionPlanner_BatteryMonitorConfiguration.png

   MissionPlanner: Battery Monitor Configuration

Enable voltage and current sensing
----------------------------------

Enter the properties your module can measure, the type of module, the
type of flight controller, and the battery capacity:

-  **Monitor:** *Voltage and Current* or *Battery Volts*
-  **Sensor:** Supported power module, or "Other"
-  **APM ver:** Flight controller (e.g. Pixhawk )
-  **Battery Capacity:** Battery capacity in mAh

The **Sensor** selection list offers a number of Power Modules
(including popular models from 3DR and AttoPilot) which you can select
to automatically configure your module. If your PM is not on the list
then you can select **Other**, and then 
:ref:`perform a manual calibration <common-power-module-configuration-in-mission-planner_calibration>` as described below.


.. _common-power-module-configuration-in-mission-planner_calibration:

Calibration
-----------

The bottom section of the the *Battery Monitor* screen allows you to
calibrate the voltage/current measurement in order to verify that the
measured voltage of the battery is correct. You can also set the
**Sensor** selection list to **Other** and use the calibration process
to configure an "unknown" power module.

To calibrate the voltage reading:

#. Check the voltage of your LiPo battery with a hand-held volt meter or
   a `power analyzer <http://www.hobbyking.com/hobbyking/store/__10080__Turnigy_130A_Watt_Meter_and_Power_Analyzer.html>`__
#. Connect your Pixhawk-series to your computer and plug in the LiPo battery
#. Check the voltage through the *Mission Planner*'s **INITIAL SETUP \|
   Optional Hardware \| Battery Monitor** screen or on the Flight Data
   screen's HUD or *Status* tab.

   .. image:: ../../../images/MPCheckVoltage.jpg
       :target: ../_images/MPCheckVoltage.jpg

If you find the voltage is not correct (i.e. if off from the hand-held
volt meter's reading by more than perhaps 0.2V) you can correct the
APM/PX4's reading by doing the following:

#. On *Mission Planner*'s **INITIAL SETUP \| Optional Hardware \|
   Battery Monitor** screen set the "Sensor" to "Other".
#. Enter the voltage according to the hand-held volt meter in the
   "Measured Battery Voltage" field
#. Press tab or click out of the field and the "Voltage Divider
   (Calced)" value will update and the "Battery voltage (Calced)" should
   now equal the measured voltage

   .. image:: ../../../images/CalibrateVoltage.png
       :target: ../_images/CalibrateVoltage.png

Using the power analyser you can also measure the current and compare to
results displayed in the Mission Planner.

.. note::

   Most current sensors are not very accurate at low currents (less
   than 3Amps). Typically you should perform current calibration at around
   10A. The exception is PMs that use hall-effect sensors, like :ref:`those from Mauch <common-mauch-power-modules>`.

This video shows the voltage and current calibration process using a
Turnigy Power Analyser.

..  youtube:: tEA0Or-1n18
    :width: 100%

Enable Low Battery Alert
------------------------

You can set *Mission Planner* to alert you verbally when your battery is
low (using a computerized voice).

Simply check the **MP Alert on Low Battery** checkbox and enter the
warning you wish to hear, the voltage level and finally the percentage
of remaining current.

.. image:: ../../../images/MP_battery_alarm_001.png
    :target: ../_images/MP_battery_alarm_001.png

.. image:: ../../../images/MP_battery_alarm_002.png
    :target: ../_images/MP_battery_alarm_002.png

.. image:: ../../../images/MP_battery_alarm_003.png
    :target: ../_images/MP_battery_alarm_003.png


Connecting power module to alternative pins
===========================================

The power module is generally plugged into the default port on the
flight controller (ie. Pixhawk). If you wish to change where the power
module is plugged into the controller, the pins used can be modified
using the BATT_VOLT_PIN and BATT_CURR_PIN parameters.

The list of available analog input pins that can be used are listed on
the Hardware Options page for each board
(:ref:`Pixhawk <common-pixhawk-overview_pixhawk_analog_input_pins>`,
:ref:`APM2 <common-apm25-and-26-overview_analog_input_pins>`).

