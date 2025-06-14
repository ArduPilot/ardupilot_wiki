Astra Smart Battery
====================

.. _astra-smart-battery:

The **Astra Smart Battery** is a smart battery based on the SBS (Smart Battery System) protocol over I2C. It is compatible with the Cube autopilot series and other ArduPilot-compatible boards with I2C connectivity.

.. note::
   ArduPilot’s `SMBUS_Generic <https://ardupilot.org/dev/docs/smart-battery.html>`__ backend is used as the basis for this battery monitor. Astra includes enhancements for reporting **State of Health (SOH)** and **Cycle Count**.

Features
--------

- SBS/SMBus compliant
- Provides:
  - Voltage
  - Current
  - Remaining capacity
  - Full charge capacity
  - Temperature
  - State of Health (SOH)
  - Cycle count
  - Serial number
    and lot more data
- Plug-and-play I2C interface
- Compatible with Cube Orange / Cube Orange+ / Cube Blue and similar autopilots

Wiring
------

Connect the battery’s I2C interface to the autopilot’s I2C port. Below is the standard Cube carrier board I2C port pinout:

.. list-table:: I2C Pinout
   :widths: 25 25 50
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - GND
     - Ground
   * - 2
     - SCL
     - I2C Clock
   * - 3
     - SDA
     - I2C Data
   * - 4
     - VCC
     - 3.3V or 5V (I/P for Isolator on the BMS)

ArduPilot Configuration
-----------------------

Use Mission Planner or your preferred GCS to configure the following parameters:

.. list-table:: Battery Monitor Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Value / Description
   * - `BATT_MONITOR`
     - `32` (or `Astra`)
   * - `BATT_I2C_BUS`
     - Set to the correct bus - 0
   * - `BATT_I2C_ADDRESS`
     - Set battery I2C Address - Default 11

Additional Info
---------------
- **Cycle Count** and **SOH** values are not shown in the Mission Planner’s status tab or anywhere else. Work in progress. Can fetch the data using 0x17 for Cycle Count and 0x4f for SOH.

See Also
--------

- :ref:`battery-monitoring`
- :ref:`smart-battery`
- :ref:`smbus-battery-monitor`

