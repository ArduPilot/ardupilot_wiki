.. _common-openpilot-revo-mini:

===================
OpenPilot Mini Revo
===================

.. image:: ../../../images/openpilot-revo-mini.jpeg
    :target: ../_images/openpilot-revo-mini.jpeg

*Images and some content courtesy of the* `LibrePilot wiki <https://librepilot.atlassian.net/wiki/spaces/LPDOC/pages/26968084/OpenPilot+Revolution>`__

.. note::

   Support for the Revo Mini will be released with Copter-3.6.

Specifications
==============

-  **Processor**

   -  STM32F405RGT6 ARM Cortex-M4 microcontroller
   -  168 Mhz/1 MB Flash
   -  32-bit failsafe co-processor

-  **Sensors**

   -  InvenSense MPU6000 IMU (accel, gyro)
   -  Honeywell HMC5883L compass
   -  MS5611 barometers

-  **Power**

   -  4.8V ~ 10V input power provided through ESC connection

-  **Interfaces**

   -  6 PWM outputs (+6 more outputs possible on FlexiIO port)
   -  1 RC input PWM/PPM (+6 more PWM inputs possible on FlexiIO port)
   -  2 analog to digital inputs for battery voltage and current monitoring (2 more possible using PWM output pins 5,6)
   -  1 serial input for GPS (+1 more possible on FlexiIO port, 1 more on Flexi port)
   -  1 I2C port on Flexi port
   -  MMCX antenna connector for integrated HopeRF RFM22B 100mW 433MHz
   -  USB port
   -  SWD Port for flashing and debugging
   
  
Flashing a Beta Firmware
========================
Official support for the Revo Mini will be released in Copter 3.6, but in the meantime you can try a release candidate firmware on the board. Flashing a release candidate firmware is only possible in Mission Planner at the moment.

Compile ArduPilot
-----------------
Compile ArduPilot from the Copter-3.6 branch. After cloning, checkout commit `d575d5e` for Copter 3.6-rc1.
::
    
    git clone https://github.com/ardupilot/ArduPilot
    cd ardupilot
    git submodule update --init --recursive
    
    git checkout Copter-3.6
    git checkout d575d5e
    
    #Assuming you have all of the dependencies.
    ./waf configure --board revo-mini
    ./waf copter
    
This will generate the file build/revo-min/bin/arducopter.apj that we will use to flash the device.
    
Enter DFU Mode
--------------
The OpenPilot Revolution Mini does not come with an ArduPilot compatible bootloader and so you'll need to flash new bootloader to the device. This only has to be done once. To flash the bootloader you must first enter DFU mode. To do this, you'll need to locate and short two pads on the device. You can short the pads in any particular way (either with a wire, solder joint, or something else). Detailed instructions are available on the `Revo Mini LibrePilot Wiki <https://librepilot.atlassian.net/wiki/spaces/LPDOC/pages/29622291/Recover+board+using+DFU>`__. A small wire is the easiest way to short the device. You can also power the device via USB first, and then short the pads if using a wire. Once you have the device in DFU mode and connected to your machine continue with the steps here.

Install dfu-util
-----------------
Linux (Ubuntu)
::
    
    sudo apt-get install dfu-util
    
OS X
::
    
    brew install dfu-util
    
Windows

Refer the the Revo Mini LibrePilot wiki above. Install the Zadig USB driver and download the `LibrePilot_dfu_flash.zip <https://librepilot.atlassian.net/wiki/download/attachments/29622291/LibrePilot_dfu_flash.zip?version=2&modificationDate=1464128116188&cacheVersion=1&api=v2>`__. Extract the zip archive and open a command prompt or PowerShell window in the directory.

Flash Bootloader
----------------
Download the `bootloader <https://github.com/ArduPilot/ardupilot/blob/master/Tools/bootloaders/revo405_bl.bin>`__ for the Revo Mini from master. With the Revo Mini connected via USB and DFU mode, open a terminal and flash the new bootloader with the following command line:

::

    sudo dfu-util -d 0483:df11 -c 1 -i 0  -a 0  -D revo405_bl.bin  -s 0x08000000

Once the flashing is complete, power cycle the board and you should see a solid green power LED and a rapidly blinking blue LED.

.. image:: ../../../images/openpilot-revo-mini-awaiting-firmware.jpg
    :target: ../images/openpilot-revo-mini-awaiting-firmware.jpg


Flash ArduPilot
---------------
Open Mission Planner and go to the Initial Setup tab. Verify that the COM port in the top right is the same as in Device Manager.

.. image:: ../../../images/openpilot-revo-mini-com-ports.png
    :target: ../images/openpilot-revo-mini-com-ports.png

Choose "Load Custom Firmware" and browse to the "arducopter.apj" file. After the flash is complete, power cycle the device.

.. image:: ../../../images/openpilot-revo-mini-load-firmware.png
    :target: ../images/openpilot-revo-mini-load-firmware.png

Congratulations! You're now running ArduCopter on the OpenPilot Revolution Mini. You can use this same process to upgrade to newer versions of ArduCopter. Compile ArduCopter and upload the .apj file to the board.

.. image:: ../../../images/openpilot-revo-mini-flashed.jpg
    :target: ../images/openpilot-revo-mini-load-flashed.jpg

Known Issues
============
At the time of writing (the release of Copter 3.6-rc1) the physical board orientation differs from the orientation in software. To fix this, simply change AHRS_ORIENTATION to YAW_180.  Test in your GCS software, as this will be rectified at some point.

Where to Buy
============

- Available from many retailers including `HobbyKing <https://hobbyking.com/en_us/openpilot-cc3d-revolution-revo-32bit-flight-controller-w-integrated-433mhz-oplink.html>`__.
