.. _common-ie650-fuelcell:

======================================
Intelligent Energy Fuel Cell 650W/800W
======================================

.. figure:: ../../../images/ie650-fuel-cell-tarrot.jpg
    :target: ../_images/ie650-fuel-cell-tarrot.jpg


This page explains how to connect Intelligent Energy's 650W and 800W fuel cells to ArduPilot.  Telemetry from the fuel cell is transmitted via serial to ArduPilot.  This gives the following feedback:

- Hydrogen tank level as a percentage.
- Battery remaining as a percentage.
- Fuel cell state.
- Error codes.


Setup
-----

Connecting the Device
+++++++++++++++++++++

Connect the UART from the 'Customer Interface' port on the fuel cell to any serial port on your flight controller.  `See device user manual <https://www.intelligent-energy.com/our-products/support/>`__ for details on where to find the 'Customer Interface'.  Only the GND, UART TX, and UART RX connections are required.  See the diagram below:

.. figure:: ../../../images/ie650_800_Connection.png
    :target: ../_images/ie650_800_Connection.png
    :width: 400px
    :align: center

Parameter Configuration
+++++++++++++++++++++++

Enable the fuel cell driver by setting :ref:`GEN_TYPE <GEN_TYPE>` to 1.

Fuel cell telemetry is transmitted over serial.  The appropriate serial port needs to be configured to receive fuel cell data.  Set the :ref:`SERIALX_PROTOCOL <SERIAL1_PROTOCOL>` parameter to 30 for generator.

The generator library utilises the battery monitor mavlink message to display fuel cell telemetry on the ground station.  Two battery monitor instances are required for one fuel cell.  One for the fuel tank and another for the battery.  Set :ref:`BATTX_MONITOR <BATT_MONITOR>` parameter to 17 for the fuel cells electrical data.  Set another :ref:`BATTX_MONITOR <BATT_MONITOR>` parameter to 18 for the hydrogen tank telemetry.

This fuel cell unit only reports battery percentage remaining and tank percentage remaining.  As such, the following parameters must be set for both tank and battery:

- :ref:`BATTX_CAPACITY <BATT_CAPACITY>` = 100 must be set to give a sensible readout of a percentage on the GCS.

- Capacity related failsafes can be set using :ref:`BATTX_LOW_MAH <BATT_LOW_MAH>`, :ref:`BATTX_CRT_MAH <BATT_CRT_MAH>`, :ref:`BATTX_ARM_MAH <BATT_ARM_MAH>`.

Reboot the flight controller after setting the parameters for the settings to take effect.

.. tip::
    In this instance, ignore the units of these parameters.  The capacities are only reported as a percentage.  As such values should be entered in the range from 1 to 100.

.. note::
    No voltage data is available for these units and a fixed voltage of 1 V is always reported.  To avoid low voltage pre-arm warnings and failsafes :ref:`BATTX_ARM_VOLT <BATT_ARM_VOLT>` = 0, :ref:`BATTX_LOW_VOLT <BATT_LOW_VOLT>` = 0, and :ref:`BATTX_CRT_VOLT <BATT_CRT_VOLT>` = 0 must be set to disable voltage related failsafes on this model.


Fuel Cell Status
----------------

The fuel cell reports status over the telemetry.  The status levels are:

- Starting
- Ready
- Running
- Fault
- Battery Only

A message will be displayed on the GCS and in the flight log whenever the fuel cell status changes.  The vehicle can only be armed when the fuel cell is reporting a status of 'Running'.


Failsafes and Pre-Arm Checks
----------------------------

Failsafes and pre-arm checks are hard-coded and based on the error codes sent by the fuel cell.  Failsafes have been separated into two groups: Low and Critical.  The error code groups are listed below.  If an error code is received from the fuel cell, the resulting failsafe action will be set by the :ref:`BATTX_FS_LOW_ACT <BATT_FS_LOW_ACT>` and :ref:`BATTX_FS_CRT_ACT <BATT_FS_CRT_ACT>` respectively.

.. Note::
    Monitoring of the fuel cell's internal error codes for pre-arm checks and failsafes are only done on the battery monitor instance allocated for the electrical telemetry.  The electrical battery monitor instance must therefore be set to trigger failsafes based on error codes.


Failsafe Low Action Error Code Group
++++++++++++++++++++++++++++++++++++

.. raw:: html

   <table border="1" class="docutils">
   <tbody>

   <tr>
   <th>Error Code</th>
   <th>Definition</th>
   </tr>

   <tr>
   <td>0x4000000</td>
   <td>Fan over current (> 0.25 A)</th>
   </tr>

   <tr>
   <td>0x100000</td>
   <td>Fuel cell's internal State is set 'stop' for > 15 s</th>
   </tr>

   <tr>
   <td>0x20000</td>
   <td>Tank pressure < 15 barg</th>
   </tr>

   <tr>
   <td>0x2000</td>
   <td>Stack 1 under temperature (< 5 degC)</th>
   </tr>

   <tr>
   <td>0x1000</td>
   <td>Stack 2 under temperature (< 5 degC)</th>
   </tr>

   <tr>
   <td>0x800</td>
   <td>Battery under voltage warning (21.6 V)</th>
   </tr>

   <tr>
   <td>0x200</td>
   <td>Fan pulse aborted</th>
   </tr>

   <tr>
   <td>0x100</td>
   <td>Stack under voltage (650 W < 17.4V, 800 W < 21.13 V)</th>
   </tr>

   <tr>
   <td>0x80</td>
   <td>Stack under voltage and battery power below threshold (< -200 W)</th>
   </tr>

   <tr>
   <td>0x10</td>
   <td>Battery charger fault</th>
   </tr>

   <tr>
   <td>0x8</td>
   <td>Battery undertemperature (< -15 degC)</th>
   </tr>

   </tbody>
   </table>


Failsafe Critical Action Error Code Group
+++++++++++++++++++++++++++++++++++++++++

.. raw:: html

   <table border="1" class="docutils">
   <tbody>

   <tr>
   <th>Error Code</th>
   <th>Definition</th>
   </tr>

   <tr>
   <td>0x80000000</td>
   <td>Stack 1 over temperature alert (>58 degC)</td>
   </tr>

   <tr>
   <td>0x40000000</td>
   <td>Stack 2 over temperature alert (>58 degC)</td>
   </tr>

   <tr>
   <td>0x20000000</td>
   <td>Battery under volt alert (<19 V)</td>
   </tr>

   <tr>
   <td>0x10000000</td>
   <td>Battery over temperature alert (>65 degC)</td>
   </tr>

   <tr>
   <td>0x8000000</td>
   <td>No fan current detected (<0.01 A)</td>
   </tr>

   <tr>
   <td>0x2000000</td>
   <td>Stack 1 over temperature critical (>57 degC)</td>
   </tr>

   <tr>
   <td>0x1000000</td>
   <td>Stack 2 over temperature critical (>57 degC)</td>
   </tr>

   <tr>
   <td>0x800000</td>
   <td>Battery under volt warning (<19.6 V)</td>
   </tr>

   <tr>
   <td>0x400000</td>
   <td>Battery over temperature warning (>60 degC)</td>
   </tr>

   <tr>
   <td>0x200000</td>
   <td>Fuel cell's internal State == start for > 30 s</td>
   </tr>

   <tr>
   <td>0x80000</td>
   <td>Tank pressure < 6 barg</td>
   </tr>

   <tr>
   <td>0x40000</td>
   <td>Tank pressure < 5 barg</td>
   </tr>

   <tr>
   <td>0x10000</td>
   <td>Fuel cell's internal saftey flags not set true</td>
   </tr>

   <tr>
   <td>0x8000</td>
   <td>Stack 1 denied start</td>
   </tr>

   <tr>
   <td>0x4000</td>
   <td>Stack 2 denied start</td>
   </tr>

   <tr>
   <td>0x400</td>
   <td>Battery under voltage (21.6 V) and master denied</td>
   </tr>

   <tr>
   <td>0x40</td>
   <td>Over voltage and over current protection</td>
   </tr>

   <tr>
   <td>0x20</td>
   <td>Invalid serial number</td>
   </tr>

   </tbody>
   </table>


Example
+++++++
This example has been provided to make it clear how the failsafe actions are set.

The battery monitors have been configured so that electrical telemetry data is on battery monitor 1 and hydrogen tank telemetry is on battery monitor 2:

- :ref:`BATT_MONITOR <BATT_MONITOR>` = 17 (electrical data)
- :ref:`BATT2_MONITOR <BATT2_MONITOR>` = 18 (tank data)

The failsafe actions that correspond to the error code groups listed above are then set on battery monitor 1 as that is the one associated with the electrical telemetry data.

- :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` = 2 (RTL)
- :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` = 1 (land)

A tank level failsafe has been set on battery monitor 2 so that the vehicle will return to land when the fuel level gets below 25%.  This is done by setting:

- :ref:`BATT2_FS_LOW_ACT <BATT2_FS_LOW_ACT>` = 2 (RTL)
- :ref:`BATT2_CAPACITY <BATT2_CAPACITY>` = 100
- :ref:`BATTX_LOW_MAH <BATT_LOW_MAH>` = 25

As previously mentioned, battery capacity failsafes can be used on this model.  Hence, as a final belt-and-braces approach a critical voltage failsafe has been set on the electrical generator monitor instance to initiate an immediate landing if battery remaining gets to 40%:

- :ref:`BATT_CRT_MAH <BATT_CRT_MAH>` = 40
- :ref:`BATT_CAPACITY <BATT_CAPACITY>` = 100


Driver Not Healthy
------------------

If you see the `Generator: Not Healthy` message in the GCS then ArduPilot has not received any data packets from the fuel cell for at least 5 seconds.  Check the following for common causes to this issue:

- Fuel cell is powered on
- Telemetry is connected to a flight controller serial port.
- The serial connection is wired correctly (TX->RX, RX->TX).
- The :ref:`SERIALX_PROTOCOL <SERIAL1_PROTOCOL>` parameter is set to generator for the correct telemetry port.


Flight Log Data
---------------

The fuel and battery levels are logged as percentage used.  They are logged under the 'BAT' data group.  Battery used will be displayed under 'CurrTot' for the electrical instance.  Fuel used will be displayed under 'CurrTot' for the fuel instance.

Fuel cell state and error messages are stored under the 'MSG' data group.
