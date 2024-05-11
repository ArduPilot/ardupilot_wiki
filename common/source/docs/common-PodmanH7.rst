.. _PodmanH7:

========
PodmanH7
========

The PodmanH7 is an open-source hardware designed and maintained by `Makermare UAV-Group <https://github.com/makermare>`_. And it is sold by a range of resellers.
This FCU is taking advantage of hardware features as much as possible to improve efficiency.
Built-in voltage and current sensors.
This flight controller have 3 SPI and 5 UART Ports.

.. image:: ../../../images/PodmanH7_ShellView.png
    :target: ../_images/PodmanH7_ShellView.png


Specifications
==============

-  **Processor**

   -  STM32H7XX microcontroller 2MB 32-bit processor

-  **Sensors**

   -  Two IMUs: BMI270, ICM42688 or IIM42652
   -  BMI270 Acc/Gyro
   -  IIM42652 Acc/Gyro
   -  ICM42688P Acc/Gyro
   -  internal vibration isolation for the first IMU
   -  SPL06 barometer

-  **Power**

   -  2S - 6S Lipo input voltage with voltage monitoring(2 power analog ports)

-  **Interfaces**

   -  14x PWM outputs DShot capable (14 PWM protocols as well as DShot)
   -  1x RC input (SBUS and PPM signals)
   -  5x UARTs/serial for GPS and other peripherals,
   -  USART2, USART3, UART4, UART8, UART7 and OTG1 have full DMA (RX and TX).
   -  2 I2C ports for external compass, airspeed, etc.
   -  I2C_ORDER: I2C2 I2C1
   -  microSDCard for logging, etc.
   -  Internal RGB LED
   -  Safety switch port
   -  Buzzer port

-  **Physical**

   -  Mounting: 30 x 30mm, Φ4mm
   -  Dimensions: 40 x 70 x 9 mm
   -  Weight: 15g

Pinout
======

.. image:: ../../../images/PodmanH7_FrontView.jpg
    :target: ../_images/PodmanH7_FrontView.jpg
On each connector the red dot indicates pin 1.
Unless noted otherwise all connectors are 1.25mm pitch.

.. image:: ../../../images/PodmanH7_ShellView2.jpg
    :target: ../_images/PodmanH7_ShellView2.jpg

UART Mapping
============

The UARTs are marked Rxn and Tn in the above pinouts. The Rxn pin is the
receive pin for UARTn. The Txn pin is the transmit pin for UARTn.

* SERIAL0 -> USB
* SERIAL1 -> UART2 (MAVLink2, DMA-enabled)
* SERIAL2 -> UART3 (MAVLink2, DMA-enabled)
* SERIAL3 -> UART4 (MAVLink1, DMA-enabled)
* SERIAL4 -> UART8 (GPS, DMA-enabled)
* SERIAL5 -> UART7 (GPS, DMA-enabled)

The Telem1 and Telem2 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

The CONS port was originally used as a debug console, but is now a
general purpose UART (debug output is now on USB).

Battery Monitoring
============
The board has two dedicated power monitor ports.
The correct battery setting parameters are dependent on
the type of power brick which is connected.

The default battery parameters are:

 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN>` 14
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN>` 15
 - :ref:`BATT_VOLT_SCALE<BATT_VOLT_SCALE>` 10.75
 - :ref:`BATT_CURR_SCALE<BATT_CURR_SCALE>` 17.0

RC Input
========

RC input is configured on the RCIN pin, at one end of the servo rail,
marked RCIN in the above diagram. This pin supports all RC
protocols. In addition there is a dedicated Spektrum satellite port
which supports software power control, allowing for binding of
Spektrum satellite receivers.

PWM Output
==========

The PodmanH7 supports up to 14 PWM outputs. First first 8 outputs (labelled
"MAIN") are controlled by a dedicated STM32F103 IO controller. These 8
outputs support all PWM output formats.

The remaining 6 outputs (labelled AUX1 to AUX6) are the "auxiliary"
outputs. These are directly attached to the STM32H7XX and support all
PWM protocols as well as DShot.

All 14 PWM outputs have GND on the top row, 5V on the middle row and
signal on the bottom row.

The 8 main PWM outputs are in 3 groups:

 - PWM 1 and 2 in group1
 - PWM 3 and 4 in group2
 - PWM 5, 6, 7 and 8 in group3

The 6 auxiliary PWM outputs are in 2 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5 and 6 in group2

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

GPIOs
=====

The 6 PWM ports can be used as GPIOs (relays, buttons, RPM etc). To
use them you need to limit the number of these pins that is used for
PWM by setting the BRD_PWM_COUNT to a number less than 6. For example
if you set BRD_PWM_COUNT to 4 then PWM5 and PWM6 will be available for
use as GPIOs.

The numbering of the GPIOs for PIN variables in ArduPilot is:

 - PWM1 50
 - PWM2 51
 - PWM3 52
 - PWM4 53
 - PWM5 54
 - PWM6 55
 - GPIO 94

Analog
=====

The PodmanH7 has 5 analog inputs

 - ADC Pin14 -> (up to +3.3V) Battery Voltage
 - ADC Pin15 -> (up to +3.3V) Battery Current Sensor
 - ADC Pin8 -> (up to +6.6V) PRESSURE SENS ADC port input
 - ADC Pin4 -> (up to +3.3V) AUX_ADC1 Sensor (requires custom carrier board)
 - ADC Pin103 -> RSSI voltage monitoring

Compass
========

The PodmanH7 has an IST8310 compass on board.
You can attach external compass using I2C2 or I2C1,
on the GPS(SERIAL4&I2C2), GPS2(SERIAL5&I2C2) and DISPLAY(I2C1) ports.

Firmware
========

Firmware for this board can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled "PodmanH7".

Loading Firmware
================

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.

There are two SWD connectors, one for FMU(STM32H7), 
and the other for IOMCU(STM32F1).

[copywiki destination="plane,copter,rover,blimp"]
