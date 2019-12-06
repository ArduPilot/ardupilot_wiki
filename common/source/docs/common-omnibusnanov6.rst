.. _common-omnibusnanov6:

===============================
Omnibus F4 Nano V6 (must be V6)
===============================

.. image:: ../../../images/omnibusNanoV6.jpg
    :target: ../_images/omnibusNanoV6.jpg
    :width: 450px

Above image and some content courtesy of `myairbot.com <https://store.myairbot.com/flight-controller/omnibus-f3-f4/omnibusf4nanov6.html>`__

.. note::

   Support for this board is available with Copter-3.6.0 (and higher)

Specifications
==============

-  **Processor**

   -  STM32F405 ARM


-  **Sensors**

   -  InvenSense MPU6000 IMU (accel, gyro)
   -  BMP280 barometer
   -  Voltage sensor


-  **Interfaces**

   -  2 UARTS
   -  4 PWM outputs
   -  RC input PWM/PPM, SBUS
   -  I2C port for external compass
   -  USB port
   -  Built-in OSD
   -  Onboard voltage sensor
   -  Onboard winbond 25Q128 for dataflash-type logging 

Where to Buy
============

- available from multiple retailers including `myairbot.com <https://store.myairbot.com/flight-controller/omnibus-f3-f4/omnibusf4nanov6.html>`__


Default UART order
==================

- SERIAL0 = console = USB
- SERIAL1 = Telemetry1 = USART1
- SERIAL2 = Telemetry2 = USART4
- SERIAL3 = GPS1 = USART6
- SERIAL4 = not assigned
- SERIAL5 = not assigned
- SERIAL6 = not assigned

Serial protocols can be adjusted to personal preferences.

Dshot capability
================

All motor/servo outputs are Dshot and PWM capable. However, mixing Dshot and normal PWM operation for outputs is restricted into groups, ie. enabling Dshot for an output in a group requires that ALL outputs in that group be configured and used as Dshot, rather than PWM outputs. The output group that must be the same (same PWM rate or Dshot, when configured as a normal servo/motor output) is: 1/2 and 3/4.

Logging
=======

Logging to on-board data flash is supported on this controller.

Versions
==============
There have been many versions of this board and many clones. This for Version 6 only

Board Connections
=================

.. image:: ../../../images/nanov6.jpg
    :target: ../_images/nanov6.jpg
    :width: 450px
    
Typical Arduplane system

GPS is attached to UART6

Telem is available at UART 1

The shared UART3/I2C pins are enabled only for I2C operation to allow external compass or digital airspeed sensor attachment.

RC input is via pad marked LED on the board and is compatible all RX serial protocols supported by ArduPilot.
The Buzzer output pad has no functionality.

Battery monitoring
==================

Supports analog voltage monitoring on VBAT pin

Set :ref:`BATT_MONITOR <BATT_MONITOR>` to 3 (= analog voltage only) and reboot

Default pin values:

:ref:`BATT_VOLT_PIN <BATT_VOLT_PIN>` = 12

:ref:`BATT_VOLT_MULT <BATT_VOLT_MULT>` = 11

Optionally add voltage and / or current monitoring using BLHeli_32-capable ESCs. See instructions :ref:`here <common-dshot>` for setting up BLHeli_32 ESC telemetry.


Flashing Firmware
=================
Usually these boards are sold pre-flashed with betaflight / INav firwares and require both firmware and bootloader to be updated if you want to use ArduPilot, as an ArduPilot-compatible bootloader is required for subsequent ArduPilot firmware-upgrade handling.

Firmware files can be found `here <https://firmware.ardupilot.org/>`__
Besides the .apj files for firmware flashing via MissionPlanner, there's also .hex files for use with various utilities like dfu-util or betaflight / iNav GUIs. You will also find a _bl.hex that contains the firmware plus the ArduPilot compatible bootloader in case it is not already present on your board. 

The provided _with_bl.hex file can be flashed using BF or iNav GUI, likely the most convenient way to get ArduPilot on your board the first time.

Alternatively, the bootloader can be flashed separately. This requires the board to be put into DFU mode. Tools like dfu-util can be used to flash the bootlader. Once the bootlader is present, all subsequent firmware updates can be done using MissionPlanner's firmware functions.

Enter DFU Mode
--------------
To do this, you'll need to locate the DFU jumper on your board. On most board flavours this is a little push button thas needs to be
pressed while connecting your board to your PC via USB(board shown below is not this board, for illustration only):


.. image:: ../../../images/omnibusf4_dfu_button.png
    :target: ../_images/omnibusf4_dfu_button.png

Install dfu-util
----------------
* Linux (Ubuntu)

  sudo apt-get install dfu-util
    
* OS X


  brew install dfu-util
    
* Windows

  Download the `dfu-util <http://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/win32-mingw32/dfu-util-static.exe>`__ to your local system, e.g., under `D:\dfu-util`.

  Rename it to `dfu-util.exe`

  Append the path of the `dfu-util.exe` to the system environment variable `Path`: "My Computer" > "Properties" > "Advanced" > "Environment Variables" > "Path". Please note that paths in the variable `Path` are seperated by semicolon `;`. This will allow dfu-util to be executed globally in command prompt.

Flash Bootloader
----------------

Bootloader binaries for the current targets can be found `here <https://firmware.ardupilot.org/Tools/Bootloaders>`__.

Download omnibusf4pro_bl.bin for this board type. With your board connected via USB and put into DFU mode, open a terminal and flash the new bootloader with the following command line:

::

    sudo dfu-util -d 0483:df11 -c 1 -i 0  -a 0  -D omnibusf4pro_bl.bin  -s 0x08000000

Once the flashing is complete, power cycle the board and you should see a solid power LED and a rapidly blinking blue LED.

.. note::
   Alternatively, board-specific bootloaders can be built from source with ./waf using the --bootloader option.

Flash ArduPilot
---------------
Open Mission Planner and go to the Initial Setup tab. Verify that the COM port in the top right is the same as in Device Manager.

.. image:: ../../../images/openpilot-revo-mini-com-ports.png
    :target: ../_images/openpilot-revo-mini-com-ports.png

Choose "Load Custom Firmware" and browse to the respective .apj file. After the flash is complete, power cycle the device.

.. image:: ../../../images/openpilot-revo-mini-load-firmware.png
    :target: ../_images/openpilot-revo-mini-load-firmware.png

Congratulations! You're now running ArduPilot on your omnibusF4. You can use this same process to upgrade to newer versions of ArduPilot. Either use MP's firmware update functionality or compile your own desired vehicle firmware from source and upload the .apj file to the board.

Compile ArduPilot
-----------------
To build your own firmware, see the instructions on setting up a build envrionment and compiling the source code:
`Building the Code <https://ardupilot.org/dev/docs/building-the-code.html>`__
