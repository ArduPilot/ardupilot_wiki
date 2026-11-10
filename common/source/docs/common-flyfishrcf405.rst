.. _common-flyfishrcf405:

[copywiki destination="plane,copter,rover,blimp,sub"]

==============
FlyFishRC F405
==============

The FlyFishRC F405 is a flight controller produced by `FlyFishRC <https://www.flyfishrc.com/>`__.

.. note::

    Due to flash memory limitations, this board does not include all ArduPilot features.
    See :ref:`Firmware Limitations <common-limited_firmware>` for details.

Features
========

* MCU - STM32F405 32-bit processor, 1024KBytes Flash
* IMU - ICM-42688-P
* Barometer - DPS310
* OSD - AT7456E
* Onboard Flash: 16MByte
* 6x UARTs
* 9x PWM Outputs (8 Motor Output, 1 LED)
* Battery input voltage: 2S-8S (7.4V - 33.6V)
* BEC 5V 3A
* BEC 10V 2.5A for video, jumper selectable to battery voltage
* 4V5 output for the receiver, also powered from USB
* Hardware SBUS inverter
* Analog RSSI input
* Switched VTX power output
* Dimensions: 36 x 36 x 8 mm
* Mounting: 30.5 x 30.5mm, M3
* Weight: 9.6g

Where to Buy
============

- Available as a stack with ESC from retailers such as `FlyMod <https://flymod.net/en/item/stack_flyfishrc_f405_v12_esc>`__

Pinout
======

.. image:: ../../../images/flyfishrcf405_pinout.jpg
   :target: ../_images/flyfishrcf405_pinout.jpg

UART Mapping
============

The UARTs are marked Rn and Tn on the board. The Rn pad is the receive pin for
UARTn. The Tn pad is the transmit pin for UARTn.

* SERIAL0 -> USB
* SERIAL1 -> USART1 (RCIN, DMA capable) on the TX1/RX1 pads
* SERIAL2 -> USART2 (RX only via the inverted SBUS pad, DMA capable)
* SERIAL3 -> USART3 (User, TX DMA capable) on the TX3/RX3 pads
* SERIAL4 -> UART4 (User) on the TX4/RX4 pads
* SERIAL5 -> UART5 (User, RX only) on the RX5 pad
* SERIAL6 -> USART6 (GPS, TX DMA capable) on the TX6/RX6 pads

UART5 has no TX pad; only RX5 is broken out on the board.

RC Input
========

RC input is configured by default on SERIAL1 (USART1) using the TX1/RX1 pads. This is
a full UART, which CRSF/ELRS requires for reliable operation.

* CRSF/ELRS connects to TX1/RX1 with :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` left at "0".
* SRXL2 connects to TX1 only, with :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` set to "4".
* SBUS must use the SBUS pad, which is wired to USART2 RX through the onboard hardware
  inverter, the only inverted port on this board. Set
  :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` to "23",
  :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to "1", and
  :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` to "0", since only one serial port can be
  used for RC input.
* FPort requires an external bi-directional inverter. See :ref:`common-FPort-receivers`.

This board has no timer-based RC input pin, so PPM is not supported.

PWM Output
==========

The FlyFishRC F405 supports up to 8 PWM/DShot motor outputs plus one serial LED output.
PWM9 is the serial LED output on the LED pad and is set to that function by default.

The PWM is in 4 groups:

* PWM 1-2      in group1
* PWM 3-5      in group2
* PWM 6-8      in group3
* PWM 9 (LED)  in group4

Channels within the same group need to use the same output rate. If any channel in a
group uses DShot then all channels in that group need to use DShot.

Battery Monitoring
==================

The board has a built-in voltage sensor and an external current sensor input on the CUR
pad. The 11:1 voltage divider supports up to 8S LiPo batteries (33.6V maximum).

The correct battery setting parameters are set by default:

* :ref:`BATT_MONITOR<BATT_MONITOR>` 4
* :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` 11
* :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` 13
* :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` 11.0
* :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` 40.0

:ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` is a starting value only
and should be calibrated against actual current once the ESC is installed.

RSSI
====

The RS pad is an analog RSSI input. :ref:`RSSI_ANA_PIN<RSSI_ANA_PIN>` is preset to "12".
Set :ref:`RSSI_TYPE<RSSI_TYPE>` = 1 to enable it. :ref:`RSSI_PIN_HIGH<RSSI_PIN_HIGH>`
defaults to 5.0V, so lower it to 3.3 if the receiver outputs a 0-3.3V RSSI voltage. For
RSSI embedded in digital RC protocols like CRSF, set :ref:`RSSI_TYPE<RSSI_TYPE>` = 3.

Compass
=======

The FlyFishRC F405 does not have a builtin compass, but you can attach an external
compass using I2C on the SDA and SCL pads.

OSD Support
===========

The FlyFishRC F405 has a built-in analog OSD using an AT7456E chip, enabled by default
(:ref:`OSD_TYPE<OSD_TYPE>` = 1). Connect the camera to the CAM pad and the video
transmitter to the VTX pad.

VTX Power Switch
================

The video transmitter power can be turned off/on using GPIO 82, which is already
assigned by default to RELAY1. Connect the video transmitter supply between the adjacent
10V pad and the SW-G pad, which RELAY1 switches to ground. It is off at boot, so the
video transmitter is not powered until the relay is turned on. This relay can be
controlled either from the GCS or using a transmitter channel, see
:ref:`common-auxiliary-functions`.

10V / BAT Jumper
================

The 10V video power pads are fed from the onboard 10V regulator, selected by a 0 ohm
resistor fitted on the "10V" side of the "BAT" / "10V" solder jumper on the right-hand
side of the board. Moving that resistor to the "BAT" side connects the pads directly to
battery voltage instead, for video transmitters that accept full pack voltage.

GPIOs
=====

The following pads are available as GPIOs:

===== ====
Pad   GPIO
===== ====
E1    50
E2    51
E3    52
E4    53
E5    54
E6    55
E7    56
E8    57
LED   58
BZ+   80
SW-G  82
===== ====

Status LEDs
===========

A row of indicator LEDs is fitted beside the USB connector, labelled MCU, GYO, 3V3, 5V
and 10V. The 3V3, 5V and 10V LEDs indicate the corresponding power rails. MCU and GYO
are the two autopilot-controlled status LEDs.

Other Pads
==========

* 4V5 - output for the receiver and GPS. It is fed through a diode from both the USB
  supply and the onboard 5V BEC, which is where the lower voltage comes from, so the 4V5
  pads stay powered when only USB is connected while all other 5V pads need a battery.
  That allows a receiver to be bound and configured, or a GPS lock acquired, without a
  battery. It is not a separate regulator, so its current comes out of the 5V BEC budget.
  Be careful not to present too much load to the USB source or voltage droop may occur.
* CUR - current sense input from the ESC.
* SW-G - switched ground for the video transmitter, controlled by RELAY1. See
  "VTX Power Switch" above.

Loading Firmware
================

Firmware for this board can be found `here <https://firmware.ardupilot.org>`__ in
sub-folders labeled "FlyFishRCF405".

Initial firmware load can be done with DFU by plugging in USB with the boot button
pressed. Then you should load the "with_bl.hex" firmware, using your favourite DFU
loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot
ground station software. Updates should be done with the \*.apj firmware files.
