TF Series LiDAR with a TTL to UAVCAN Converter
================================================

This page describes how to connect a Benewake TF-series LiDAR to a Pixhawk
flight controller through a TTL to UAVCAN converter, configure ArduPilot, and
verify altitude-hold and obstacle-avoidance data.

Applicable products include TF-Luna, TFmini-S, TFmini-Plus, TF02-Pro, and TF03.
The example below uses a TFmini-S with a TTL interface.

Preparing the test setup
------------------------

The following equipment is required:

* Windows PC
* 4-pin Dupont cable
* TF-series LiDAR with a TTL interface
* Pixhawk flight controller
* TTL to UAVCAN converter
* Mission Planner
* ArduPilot firmware, such as ``apm_fmvu3.px4`` for the example controller

Install ArduPilot firmware
--------------------------

Open Mission Planner and select **Initial Setup -> Install Firmware -> Load
custom firmware**. Select ``apm_fmvu3.px4`` and follow the prompts. When the
firmware upload succeeds, click **Connect** in the upper-right corner.

Pixhawk and converter connection
--------------------------------

Connect the Pixhawk CAN interface to the TTL to UAVCAN converter, connect the
TF-series LiDAR to the converter TTL port, and connect the Pixhawk to the PC
running Mission Planner. The example below uses CAN1.

.. image:: images/tf-pixhawk-uavcan-wiring.png
   :alt: Pixhawk and TTL to UAVCAN converter wiring
   :align: center
   :width: 80%

The wiring diagram is for reference. Follow the pinout and power requirements
of the specific Pixhawk, converter, and LiDAR models in use.

Altitude hold configuration
---------------------------

In Mission Planner, open **Config/Tuning -> Full Parameter List** (or **All
Parameters**), search for each parameter below, enter the value, and select
**Write Params**.

.. list-table:: Altitude hold parameters
   :header-rows: 1
   :widths: 25 10 65

   * - Parameter
     - Value
     - Description
   * - ``CAN_P1_DRIVER``
     - 1
     - Enable CAN1. Use the appropriate CAN2 driver parameter when the converter is connected to CAN2.
   * - ``RNGFND1_TYPE``
     - 24
     - Use the UAVCAN rangefinder driver.
   * - ``RNGFND1_ADDR``
     - 16
     - Default decimal node address of the TTL to UAVCAN converter.
   * - ``RNGFND1_MIN_CM``
     - 10
     - Minimum valid distance in centimetres. Set it above the LiDAR blind zone.
   * - ``RNGFND1_MAX_CM``
     - 700
     - Maximum valid distance in centimetres. Set it below the effective LiDAR range.
   * - ``RNGFND1_GNDCLEAR``
     - 15
     - Sensor height above ground in centimetres when the vehicle is landed. Keep it above the blind zone.
   * - ``RNGFND1_ORIENT``
     - 25
     - The sensor faces downward.
   * - ``PRX1_TYPE``
     - 0
     - Do not use the proximity sensor for this altitude-hold-only configuration.

After writing the parameters, reboot the Pixhawk using **Ctrl+F -> Reboot
Pixhawk**, then reconnect.

Altitude data verification
--------------------------

Open **Flight Data** in Mission Planner. Double-click a numeric data field,
select ``rangefinder1``, and confirm that the displayed value changes when the
LiDAR-to-ground distance changes.

.. image:: images/mission-planner-rangefinder.png
   :alt: Mission Planner rangefinder data display
   :align: center
   :width: 90%

Obstacle avoidance configuration
--------------------------------

Use the following configuration when the LiDAR is mounted facing forward and
is intended for proximity-based obstacle detection.

.. list-table:: Obstacle avoidance parameters
   :header-rows: 1
   :widths: 25 10 65

   * - Parameter
     - Value
     - Description
   * - ``AVOID_BEHAVE``
     - 1
     - Stop when an obstacle is detected.
   * - ``AVOID_MARGIN``
     - 3
     - Avoidance margin in GPS-enabled mode, in metres.
   * - ``AVOID_DIST_MAX``
     - 3
     - Maximum avoidance distance without GPS, in metres.
   * - ``AVOID_ENABLE``
     - 2
     - Enable obstacle avoidance using the proximity sensor.
   * - ``CAN_P1_DRIVER``
     - 1
     - Enable CAN1 for the converter.
   * - ``RNGFND1_TYPE``
     - 24
     - Use the UAVCAN rangefinder driver.
   * - ``RNGFND1_ADDR``
     - 16
     - Default decimal node address of the converter.
   * - ``RNGFND1_MIN_CM``
     - 10
     - Minimum valid distance in centimetres; keep it above the blind zone.
   * - ``RNGFND1_MAX_CM``
     - 700
     - Maximum valid distance in centimetres; keep it below the effective range.
   * - ``RNGFND1_GNDCLEAR``
     - 15
     - Mounting clearance in centimetres; use the actual installation height.
   * - ``RNGFND1_ORIENT``
     - 0
     - The sensor faces forward.
   * - ``PRX1_TYPE``
     - 4
     - Use the rangefinder as the obstacle-avoidance input.

Select **Write Params**, reboot the Pixhawk, and reconnect. If Mission Planner
reports ``Bad LiDAR Health``, check the wiring, converter power, sensor power,
and CAN driver selection.

Proximity data verification
---------------------------

Press **Ctrl+F** in Mission Planner and select **Proximity**. The green numbers
represent the distances used by the obstacle-avoidance function. They are not
necessarily a direct real-time display of every raw LiDAR reading.

.. image:: images/mission-planner-proximity.png
   :alt: Mission Planner proximity display
   :align: center
   :width: 90%

Troubleshooting checklist
-------------------------

* Confirm that the LiDAR and converter are powered and that the supply meets the product manual requirements.
* Confirm that the Pixhawk CAN port matches the enabled CAN driver parameter.
* Confirm that ``RNGFND1_ADDR=16`` matches the converter node address.
* Confirm that ``RNGFND1_ORIENT`` matches the physical mounting direction: 25 for downward and 0 for forward.
* Confirm that ``RNGFND1_MIN_CM`` is larger than the LiDAR blind zone and ``RNGFND1_MAX_CM`` is smaller than the effective measurement range.
* Reboot the Pixhawk after changing parameters and before testing again.

The values in this page are example values. Adjust range limits, ground
clearance, and avoidance margins to the actual LiDAR model, mounting position,
and vehicle requirements.
