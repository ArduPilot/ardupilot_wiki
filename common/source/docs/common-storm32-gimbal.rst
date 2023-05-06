.. _common-storm32-gimbal:

=========================
STorM32 Gimbal Controller
=========================

The STorM32-BGC is a relatively low-cost 3-axis brushless gimbal
controller that can communicate with ArduPilot (Copter, Plane and Rover)
using MAVLink or a proprietary serial protocol.

- SToRM32 gimbals with I2C setups and firmware v0.96 should use the SToRM32 Serial driver
- SToRM32 NT gimbals running firmware above v0.96 should use the SToRM32 MAVLink driver

Where to buy
============

Please refer to the `STorM32-BGC wiki pages <http://www.olliw.eu/storm32bgc-wiki/Main_Page>`__ for more detailed information including where the gimbals can be purchased.

.. warning::

    Some v1.3x boards has been found to cause significant RF interference on the 433mhz and 915mhz band.
    Use with caution, if you are using either 433/915mhz control or telemetry.

Connecting the gimbal to the autopilot
======================================

.. image:: ../../../images/pixhawk_SToRM32_connections.jpg
    :target: ../_images/pixhawk_SToRM32_connections.jpg

Connect one of the  autopilot's serial port's TX, RX and GND pins to the gimbal's UART port as shown above.  The autopilot's serial port's VCC, RTS and CTS pins should not be connected

.. _common-storm32-gimbal_configuring_the_gimbal:

Setup if using MAVLink protocol
===============================

In addition to the regular gimbal configuration described on the
`STorM32-BGC wiki <http://www.olliw.eu/storm32bgc-wiki/Getting_Started>`__, the
MAVlink heartbeats should be enabled through OlliW's o323BGCTool's
**Tools \| Expert Tool** screen as shown below.

.. image:: ../../../images/SToRM32_enableMavlink.png
    :target: ../_images/SToRM32_enableMavlink.png

Using a ground station (e.g. Mission Planner) set the following parameters.  These setting assume Autopilot's serial2 is being used.  If another serial port is being used replace the "2" in the parameter name with the appropriate serial port number.

-  :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115 (115200 bps).
-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 1 (MAVLink1) or 2 (MAVLink2)
-  Optionally set :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` = 0 to disable serial flow control

.. image:: ../../../images/SToRM32_MP_Serial2Baud_new.png
    :target: ../_images/SToRM32_MP_Serial2Baud_new.png

if the first mount is being used, set the following parameters:

- :ref:`MNT1_TYPE <MNT1_TYPE>` = 4 (SToRM32 MAVLink) and reboot the autopilot
- set :ref:`MNT1_YAW_MIN <MNT1_YAW_MIN>`, :ref:`MNT1_YAW_MAX <MNT1_YAW_MAX>`, :ref:`MNT1_PITCH_MIN <MNT1_PITCH_MIN>`, :ref:`MNT1_PITCH_MAX <MNT1_PITCH_MAX>`, :ref:`MNT1_ROLL_MIN <MNT1_ROLL_MIN>`, :ref:`MNT1_ROLL_MAX <MNT1_ROLL_MAX>` to match your gimbal's range.
- Optionally set :ref:`RC6_OPTION <RC6_OPTION>` = 213 to control the gimbal's pitch from the transmitter's ch6 tuning knob.

The screenshot below shows a setup in which the gimbal has:

- 360 of yaw rotation (:ref:`MNT1_YAW_MIN <MNT1_YAW_MIN>` = -180, :ref:`MNT1_YAW_MAX <MNT1_YAW_MAX>`  = 179)
- 60 degrees (both left and right) of roll (:ref:`MNT1_ROLL_MIN <MNT1_ROLL_MIN>` = -60, :ref:`MNT1_ROLL_MAX <MNT1_ROLL_MAX>`  = +60)
- Can point straight down (:ref:`MNT1_PITCH_MIN <MNT1_PITCH_MIN>` = -9000)
- Can point straight up (:ref:`MNT1_PITCH_MAX <MNT1_PITCH_MAX>` = +90)
- Gimbal's pitch is controlled by the transmitter's channel 6 tuning knob

.. image:: ../../../images/SToRM32_MP_MountParams.png
    :target: ../_images/SToRM32_MP_MountParams.png

Setup if using SToRM32 Serial protocol
======================================

To use the serial protocol use all the same settings as above except:

-  When :ref:`Configuring the Gimbal <common-storm32-gimbal_configuring_the_gimbal>` controller set the "MAVLink configuration" parameter to "no heartbeat"
-  :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 8 (SToRM32 Gimbal Serial).  If another serial port is connected to the gimbal replace "2" with the serial port number
-  :ref:`MNT1_TYPE <MNT1_TYPE>` = 5 (SToRM32 Serial)

Control and Testing
===================

See :ref:`Gimbal / Mount Controls <common-mount-targeting>` for details on how to control the gimbal using RC, GCS or Auto mode mission commands

Resistor issue on some boards
=============================

Some in-depth analysis `here on rcgroups <https://www.rcgroups.com/forums/showthread.php?2494532-Storm32-with-Pixhawk-over-serial-connection/page5>`__
turned up that some STorM32 boards need resistor #4 (shown in pic below)
shorted (i.e. a wire soldered over the top of the resistor to turn it
into a regular wire) in order for the gimbal controllers messages to get
through to the Pixhawk.

.. image:: ../../../images/Gimbal_SToRM32_resistorFix.jpg
    :target: ../_images/Gimbal_SToRM32_resistorFix.jpg
