.. _common-dshot-blheli32-telemetry:

====================================
BLHeli_32 and BLHeli_S ESC Telemetry
====================================

If the ESC has this capability, it allows monitoring and logging of performance data that previously required additional sensors (like power modules and RPM sensors). The detailed data provided by every ESC allows real-time decisions and individual ESC or motor performance tuning and failure analysis. Note that a given ESC may or may not have a specific sensor's data transmitted via telemetry. It is common for 4 in 1 escs to provide voltage and current sensors but not transmit the data via telemetry, but rather by direct connection to the autopilot. Check the ESC data sheet and connection information for details.
 
.. note:: ArduPilot does not currently support the polling of the ESCs for telemetry data via throttle idle messages over the signal line in non DShot protocols.

Connecting your ESCs for use with Dshot protocol and BLHeli_32/BLHeli_S features
================================================================================

.. image:: ../../../images/dshot-pixhawk.jpg
    :target: ../_images/dshot-pixhawk.jpg
    :width: 600px

Connect all ESC's telemetry wires to a single serial port's RX pin on the autopilot (above diagram uses Serial5 as an example). ESC telemetry is currently only supported with BLHeli_32 ESCs. A pin or wire for ESC telemetry is pre-soldered on most BLHeli_32 ESCs. If the wire isn't pre-soldered you will need to solder it yourself. Pinouts for serial ports on The Cube can be found `here <https://ardupilot.org/copter/docs/common-pixhawk-serial-names.html>`__.

Set the following parameters to enable BLHeli_32 telemetry feedback to a autopilot's serial port:

- :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` = 1 to enable automatic mapping of multirotor motors for BLHeli_32 pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. If using BLHeli_32 ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:

- :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` : a bitmap used to enable BLHeli_32 pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable pass-through and telemetry on (if available in ESC).

- :ref:`SERVO_BLH_OTYPE<SERVO_BLH_OTYPE>` : This needs to be set to the protocol being used for the DShot protocol being used on those additional outputs if not the same as the normal copter style motor outputs.

- :ref:`SERIAL5_PROTOCOL <SERIAL5_PROTOCOL>` 16 (= ESC telemetry). This assumes serial port 5 is used. Adjust the serial port's protocol parameter to 16 , for the serial port whose RX input is connected to the ESC(s) telemetry pad. The correlation between serial port numbering and UART physical ports for you autopilot should be documented in its description page linked :ref:`here <common-autopilots>`.

- :ref:`SERVO_BLH_TRATE <SERVO_BLH_TRATE>` defaults to 10. this enables telemetry at a 10hz update rate from the ESC.

- :ref:`SERVO_BLH_POLES <SERVO_BLH_POLES>` defaults to 14 which applies to the majority of brushless motors. Adjust as required if you're using motors with a pole count other than 14 to calculate true motor shaft RPM from ESC's e-field RPM.

Logging and Reporting
---------------------

The autopilot requests status information from one ESC at a time, cycling between them. This information is logged to the onboard log's ESCn messages and can be viewed in any :ref:`ArduPilot compatible log viewer <common-logs>`.

- RPM
- Voltage
- Current
- Temperature
- Total Current

The RCOU messages are also written to the onboard logs which hold the requested output level sent to the ESCs expressed as a number from 1000 (meaning stopped) to 2000 (meaning full output).

This data can also be viewed in real-time using a ground station.  If using the Mission Planner go to the Flight Data screen's status tab and look for esc1_rpm.

.. image:: ../../../images/dshot-realtime-esc-telem-in-mp.jpg
    :target: ../_images/dshot-realtime-esc-telem-in-mp.jpg
    :width: 450px

.. note::

   Sending BLHeli_32 telemetry data to the GCS requires the telemetry connection use MAVLink2.  ArduPilot uses MAVLink2 by default on the USB port but if another port is used it may be necessary to set the SERIALx_PROTOCOL parameter to 2 (where "x" is the serial port number used for the telemetry connection).

In addition, some telemetry values can be displayed on the integrated :ref:`on-board OSD <common-osd-overview>`, if your autopilot has one.

.. _esc-telemetry-based-battery-monitor:

Use as Battery Monitor
======================

By setting a battery monitor instance to BLHeli32 ESC  type (for example :ref:`BATT2_MONITOR<BATT2_MONITOR>` = 9), all connected BLHeli32 ESCs with connected telemetry wiring to the configured autopilot serial port, will be aggregated as a single source. The voltages reported will be averaged, the currents totaled, and the consumed current accumulated.

.. _bidir-dshot:

Bi-directional Dshot
====================

Newer versions of BLHeli_32 (32.7) and BLHeli_S (16.73) support returning motor RPM values over the Dshot signal line. Supporting bi-directional Dshot requires exclusive use of one or more DMA channels and thus not all versions of ArduPilot support it. Versions that support bi-directional Dshot natively are `BeastH7`, `BeastF7` and `KakuteF7Mini`, other firmware versions end in "-bdshot" to indicate support for bi-directional Dshot.

Setup
-----

First ensure that you have an appropriate version of BLHeli_32 or BLHeli_S installed on your ESCs. The majority of ESCs do not come pre-installed with these versions. The official 32.7 version of BLHeli_32 supports bi-directional Dshot. Official versions of BLHeli_S do not support bi-directional Dshot, you will need to either buy a version from `JESC <https://jflight.net/index.php?route=common/home&language=en-gb>`__ or use the unofficial version from `JazzMaverick <https://github.com/JazzMaverick/BLHeli/tree/JazzMaverick-patch-1/BLHeli_S%20SiLabs/Hex%20files%20%2016.73>`__. If you try and enable bi-directional Dshot with the wrong firmware version then unpredictable motor operation can occur.

Set the following parameters to enable BLHeli_32 and BLHeli_S bi-directional Dshot:

- :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` = 1 to enable automatic mapping of multirotor motors for BLHeli_32 pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. If using BLHeli_32 ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:

- :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` : a bitmap used to enable BLHeli_32 pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable pass-through and telemetry on (if available in ESC).

- :ref:`SERVO_BLH_BDMASK <SERVO_BLH_BDMASK>` : a bitmap used to enable BLHeli_32 or BLHeli_S bi-directional Dshot support. On flight controllers without IOMCU this would normally be set to 15 to indicate four active channels. On flight controllers with an IOMCU this can be set to 3840 to indicate four active AUX channels (bi-directional Dshot will only work on the AUX outputs). The BeastH7 only supports channels 1 and 4 for bi-directional dshot (mask set to 9).

- :ref:`SERVO_BLH_OTYPE<SERVO_BLH_OTYPE>` : This needs to be set to the protocol being used for the DShot protocol being used on those additional outputs if not the same as the normal copter style motor outputs.

- :ref:`SERVO_BLH_POLES <SERVO_BLH_POLES>` defaults to 14 which applies to the majority of brushless motors. Adjust as required if you're using motors with a pole count other than 14 to calculate true motor shaft RPM from ESC's e-field RPM (small motors might have 12 poles).


