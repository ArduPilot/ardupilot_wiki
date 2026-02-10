.. _readme_file:

=====================
HWDEF Readme.md Guide
=====================

Every autopilot board port should include a Readme.md file explaining the board's features, pinouts, a required setup information. This file is converted later into a Wiki page that users will reference when deciding on an autopilot to use, and when setting up the autopilot in a system. The required sections are:

.. note:: Markdown linting is being enforced now on Readme.me files (ie. not embedded HTML, tables must be markdown format, urls in proper formaat, no improper whire spacing,etc.)

- :ref:`Introduction <readme-intro>`
- :ref:`Features/Specifications <readme-specs>`
- :ref:`Where to Buy <readme-buy>`
- :ref:`Pinout <readme-pinout>`
- Wiring Diagram (optional)
- :ref:`UART Mapping <readme-uart>`
- :ref:`PWM Outputs <readme-pwm>`
- :ref:`RC Input <readme-rc>`
- :ref:`OSD Support (if applicable) <readme-osd>`
- :ref:`VTX Control (if applicable) <readme-vtx>`
- :ref:`Camera Switch (if applicable) <readme-cam>`
- WIFI module (if applicable)
- :ref:`GPIOs <readme-intro>`
- :ref:`RSSI/Airspeed/Analog Pins <readme-rssi>`
- :ref:`Battery Monitor <readme-batt>`
- :ref:`Compass <readme-compass>`
- :ref:`Firmware <readme-firmware>`
- :ref:`Loading Firmware <readme-loading>`
- Other links (ie OEM user manual, schematics if open source,etc., if applicable)

.. note:: when porting a board that has been targeted for Betaflight, concurrently or originally, be sure that the UART protocols match the Betaflight reference design, if possible, to minimize transitioning to ArduPilot.

A good example is the `SpeedyBeeF405WING Readme.md <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/Readme.md>`_.

`
A template for a fictitious autopilot is `HERE <https://raw.githubusercontent.com/ArduPilot/ardupilot_wiki/refs/heads/master/misc/readme_template.md>`__ which contains the proper formatting and sections (if applicable) that only requires text editing.

.. _readme-intro:

Introduction
============
A brief description with link to OEms website and a **photo** of the autopilot.

.. _readme-specs:

Features/Specifications
=======================
A list of specifications:

- Processor(s) and OSD chip if included
- Flash for logging if included
- List of Sensors
- External ports(UARTs,BECs,power/device switches,etc.)
- Telemetry modules
- Physical dimensions/weight

.. _readme-buy:

Where to Buy
============
Vendor links for purchasing

.. _readme-pinout:

Pinout
======
**Clear image showing all pin names and/or connectors**. Connector pinouts should be marked or provided via table(s).

.. _readme-uart:

UART Mapping
============
- Pin designation
- Table with ArduPilot serial port mapping, UART, default protocol

preferred table format example:

  =======   =========  ========  ===========
  Serial#   Protocol   Port       Notes
  =======   =========  ========  ===========
  SERIAL0   OTG1       USB
  SERIAL1   Telem1     USART2    DMA Enabled
  SERIAL2   Telem2     USART3    DMA Enabled
  SERIAL3   GPS1       USART1    DMA Enabled
  SERIAL4   GPS2       UART4     DMA Enabled
  SERIAL5   RCin       UART8     DMA Enabled
  SERIAL7   ESC TELEM  UART6     RX6 only on ESC connector
  =======   =========  ========  ===========

- note if any UART only pins out one pin or if a pin is tied to another pin (common for SBUS)

.. _readme-pwm:

PWM Output
==========
Protocol capability of each motor/servo output including its pin name. Timer groupings with warning about all outputs in a group must be the same protocol. PWM serial LEDs should be designated here.

.. _readme-rc:

RC Input
========
This is the most complex section due to the multitude of ways boards have their RC inputs configured in the board design and that F4s vs H7 have different capabilities.

The important point is that information should be provided as to where to connect each type of receiver and what ArduPilot parameters must be set for each.

Also, if a board alternate config can be used, it should be mentioned.

Broad group names can be used, ie " Using the RCin pin will support all unidirectional RC protocols." Which would cover PPM, SBUS, iBus, PPM-Sum, DSM,DSM2,DSM-X,SRXL, and SUM-D".

Bi-directional protocols like CRSF/ELRS,SRXL2,IRC Ghost, and FPort need specific instructions with at least CRSF/ELRS instructions and  a link to :ref:`common-rc-systems`.

.. _readme-osd:

OSD Support
===========
Note if onboard OSD is provided. hwdef should already set :ref:`OSD_TYPE<OSD_TYPE>`. Adding :ref:`OSD_TYPE2<OSD_TYPE2>` 5  in the defaults.param file will enable DisplayPort operation simultaneously with analog OSD. If a UART is set to DisplayPort by default, so should the OSD type.

.. _readme-vtx:

VTX Control
===========
Info on GPIO pin #, board pin name, and if a RELAY is pre-configure.

.. note:: its good practice to have the boot block and initial state of the power switch to be OFF.

.. _readme-cam:

Camera Switch
=============
Info on GPIO pin #, board pin names, and if a RELAY is pre-configured.

.. _readme-gpios:

GPIOs
=====
If User GPIOs are defined, list them here.

.. _readme-rssi:

RSSI/Airspeed/Analog Pins
=========================
If an analog RSSI, general purpose analog inputs, or Airspeed pin is defined, give its GPIO pin #.

.. _readme-batt:

Battery Monitor
===============
If a battery monitor is defined in the hwdef, its default parameter and voltage range info should be provided. If DroneCAN or SMBus type, info about its setup.

Example:

.. code::

   The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input. The voltage sensor can handle up to 8S LiPo batteries.

   The default battery parameters are:

   * :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
   * :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 12
   * :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
   * :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11
   * :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 30.2 (will need to be adjusted for whichever current sensor is attached)


.. _readme-compass:

Compass
=======
if it does not have a compass, but has I2C pads/pins:

*The {autopilot name here} does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.*

if it does have a compass:

*The {autopilot name here} has a built-in compass. Due to potential interference, the autopilot is usually used with an external I2C compass as part of a GPS/Compass combination.*

.. _readme-firmware:

Firwmare
========
*Firmware for {autopilot name here} can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled "{autopilot config name}"*.

.. note: can be included in next section if desired

.. _readme-loading:

Loading Firmware
================
if preloaded with ArduPilot firmware:

*The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.*

if not:

*This board does not come with ArduPilot firmware pre-installed. Use instructions here to load ArduPilot the first time :ref:`common-loading-firmware-onto-chibios-only-boards`.*
