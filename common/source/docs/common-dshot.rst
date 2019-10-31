.. _common-dshot:

===============================
DShot and BLHeli_32 ESC Support
===============================

[copywiki destination="copter,plane,rover"]

This articles describes how to setup and use features supported by recent BLHeli ESC firmware.

- DShot fast digital ESC protocol support
- BLHeli_32 pass-thru ESC configuration and firmware flashing
- BLHeli_32 ESC telemetry support

Where to buy
============

A `search for "BLHeli32 shopping" <https://www.google.com/search?q=blheli32&tbm=shop>`__ turns up many compatible ESCs.  Look for an ESC which includes the telemetry wire connector like the `HolyBro Tekko32 shown below <https://shop.holybro.com/holybro-tekko32-esc35a_p1074.html>`__

.. image:: ../../../images/dshot-telemwire.png
    :target: https://shop.holybro.com/holybro-tekko32-esc35a_p1074.html

*image courtesy of holybro.com*

.. note::

   These features are available with Copter-3.6, Plane-3.9 and Rover-3.5 (or higher) using the ChibiOS firmware for STM32 based flight boards.

.. note::
   Recently there is a growing number of proprietary and non-proprietary 16 / 32 bit ESCs with firmware that support DShot and other digital ESC protocols, but not BLHeli_32-specific features like passthrough and telemetry. See your ESC's manual for further detail on supported features.


.. note::
   ArduPilot firmware supports the pass-through protocol with up-to-date BLHeli_32 firmware and BLHeliSuite32 only.


Connecting your ESCs for use with Dshot protocol and BLHeli_32 features
=======================================================================

.. image:: ../../../images/dshot-pixhawk.jpg
    :target: ../_images/dshot-pixhawk.jpg
    :width: 600px

DShot and BLHeli_32 features are currently only supported on the "FMU" outputs of your flight controller. Boards with IO coprocessors like :ref:`Pixhawk <common-pixhawk-overview>` and :ref:`The Cube <common-thecube-overview>` provide DShot and BLHeli_32 support only on the AUX OUT ports that are directly driven by the board's main processor.
For :ref:`Pixracer <common-pixracer-overview>` and :ref:`other boards <common-autopilots>` without a separate IO coprocessor, all PWM outputs can be used.

.. note::
   Output ports usually are arranged in groups of two or three using a common timer. It is not possible to mix different output types (Dshot and traditional servo-type PWM) within one common timer group. See your respective board's hardware instructions for further detail on arranging DShot-type and traditional servo-type PWM outputs.


DShot ESC protocol
==================

Dshot is a digital ESC protocol. In contrast to traditional servo-type PWM it allows fast, high resolution digital communication. This opens the door for more precise vehicle control. This is especially useful in multirotor and quadplane applications.

..  note::
   Only try DShot on ESCs that are known to support it or you will get unpredictable results. Reverse thrust is supported in 4.0 and later firmware versions.

The DShot ESC protocol's key advantages are:

- all values sent to the ESC are checksum-protected
- clock differences between the ESC and flight controller don't affect flight performance
- no need to do any ESC throttle range calibration
- very high protocol frame rates are supported

..  note::
   ArduPilot is currently supporting DShot output on stable releases of copter and plane firmware versions only.


Technical detail
----------------

The DShot protocol can run at different speeds. ArduPilot supports four speeds:

- DShot150 at 150kbaud (recommended)
- DShot300 at 300kbaud
- DShot600 at 600kbaud
- DShot1200 at 1200kbaud

We recommend using the lowest baud rate, DShot150, as it is the most reliable protocol (lower baudrates are less susceptible to noise on cables). Higher values will be beneficial once ArduPilot's main loop rate is capable of speeds above 1kHz.

DShot sends 16 bits per frame, allocated as follows:

- 11 bits for the throttle level
- 1 bit for telemetry request
- 4 bits for CRC (simple XOR)

This gives a good throttle resolution, with support for ESC telemetry feedback, if available from the ESC. See below for more information on ESC telemetry.


Configuring DShot ESC protocol output
-------------------------------------

For using DShot with multirotor motors, set :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` or :ref:`Q_M_PWM_TYPE <Q_M_PWM_TYPE>` on quadplanes to **4** (= DShot150).

For using DShot on non-multirotor motors like traditional fixed wings' main motors (SERVOn_FUNCTION = 70 throttle, 73 throttle left and / or 74 throttle right), specify the throttle outputs using :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` and set :ref:`SERVO_BLH_OTYPE <SERVO_BLH_OTYPE>` to **4** (= DShot150).

- on Pixhawk and Cube boards:

  - do not use channels 1-8 for DShot ESC's. Turn off :ref:`SERVO1_FUNCTION <SERVO1_FUNCTION>` to :ref:`SERVO8_FUNCTION <SERVO8_FUNCTION>` OR set them to something other than motor or throttle functions.
  
  - set the auxillary channels to their appropriate functions (:ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>` to :ref:`SERVO14_FUNCTION <SERVO14_FUNCTION>`). For quadcopters quadplanes, these parameters will be 33, 34, 35, and 36 for channels 9-12 (Aux 1-4).
  - When using more than the first 4 Aux ports for DShot ESC's, set :ref:`BRD_PWM_COUNT <BRD_PWM_COUNT>` to 6.


BLHeli_32 Pass-Through Support
==============================

BLHeli_32 pass-through protocol allows you to configure and upgrade your ESCs without having to disconnect them from your vehicle. You can plug a USB cable into your flight controller and run the BLHeliSuite32 software for Windows to configure your ESCs. ArduPilot firmware supports the pass-through protocol with BLHeli_32 only.

The following section shows how to setup BLHeli_32 pass-through support:
------------------------------------------------------------------------

..  youtube:: np7xXY_e5sA
    :width: 100%


To enable BLHeli_32 pass-through you need to set the following parameters and reboot your flight controller:

- Set :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` to 1 to enable automatic mapping of multirotor motors for BLHeli_32 pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. if using BLHeli_32 ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:
  
- Use :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` to enable BLHeli_32 pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable BLHeli_32 pass-through and telemetry on.

- Set :ref:`SERVO_BLH_PORT <SERVO_BLH_PORT>` to specify the flight controller's port used to connect to your PC running BLHeliSuite32 for ESC configuration. It defaults to USB and likely does not need to be altered. Beware that this does NOT specify the serial port used for the ESC's telemetry feedback to your flight controller!

Now connect a USB cable to your flight controller and use BLHeliSuite32 on Windows to connect. Select "BLHeli32 Bootloader (Betaflight/Cleanflight)" from the interfaces menu.

.. image:: ../../../images/blhelisuite32.jpg
    :target: ../_images/blhelisuite32.jpg


BLHeli_32 ESC telemetry feedback
================================

This allows monitoring and logging of performance data that previously required additional sensors (like power modules and RPM sensors). The detailed data provided by every ESC allows real-time decisions and indidvidual ESC or motor performance tuning and failure analysis.

Connect all ESC's telemetry wires to a single serial port's RX pin on the flight controller (above diagram uses Serial5 as an example). ESC telemetry is currently only supported with BLHeli_32 ESCs. A pin or wire for ESC telemetry is pre-soldered on most BLHeli_32 ESCs. If the wire isn't pre-soldered you will need to solder it yourself. Pinouts for serial ports on The Cube can be found `here <http://ardupilot.org/copter/docs/common-pixhawk-serial-names.html>`__.

Set the following parameters to enable BLHeli_32 telemetry feedback to a flight controller's serial port:

- :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` = 1 to enable automatic mapping of multirotor motors for BLHeli_32 pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. If using BLHeli_32 ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:
  
- :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` : a bitmap used to enable BLHeli_32 pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable pass-through and telemetry on (if available in ESC).

- :ref:`SERIAL5_PROTOCOL <SERIAL5_PROTOCOL>` 16 (= ESC telemetry). This assumes serial port 5 is used. Adjust the serial port's protocol parameter to 16 , for the serial port that your ESC telemetry wire is connected to, as required. The correlation between serial port numbering and UART physical ports for you flight controller should be documented in its description page linked :ref:`here <common-autopilots>`.

- :ref:`SERVO_BLH_TRATE <SERVO_BLH_TRATE>` defaults to 10. this enables telemetry at a 10hz update rate from the ESC.

- :ref:`SERVO_BLH_POLES <SERVO_BLH_POLES>` defaults to 14 which applies to the majority of brushless motors. Adjust as required if you're using motors with a pole count other than 14 to calculate true motor shaft RPM from ESC's e-field RPM.

The flight board requests telemetry from only one ESC at a time, cycling between them. The following data is logged in the ESCn log messages in your dataflash log. This can be viewed in any ArduPilot dataflash log viewer.

- RPM
- Voltage
- Current
- Temperature
- Total Current

This data can also be viewed in real-time using a ground station.  If using the Mission Planner go to the Flight Data screen's status tab and look for esc1_rpm.

.. image:: ../../../images/dshot-realtime-esc-telem-in-mp.jpg
    :target: ../_images/dshot-realtime-esc-telem-in-mp.jpg
    :width: 450px

.. note::
   Sending BLHeli_32 telemetry data to your GCS requires using mavlink2 on your GCS connection. While on current ArduPilot firmware the USB port defaults to mavlink2, it might require adjusting the protocol setting when using a different port for GCS connection.

In addition, some telemetry values can be displayed on the integrated :ref:`on-board OSD <common-osd-overview>`, if your flight controller has one.
