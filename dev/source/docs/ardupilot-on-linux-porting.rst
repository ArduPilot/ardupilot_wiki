.. _ardupilot-on-linux-porting:

=========================================
Porting to a new flight controller board
=========================================

This section is for topics related to running ArduPilot directly on Linux boards.
See this :ref:`page for porting to boards running ChibiOS <porting>`.

.. note:: Any firmware customization of ArduPilot code must abide by the terms of the GPL3.0+ open source code license. ArduPilot also reminds developers and manufacturers to adhere to the appropriate trademark and copyright laws when developing new autopilots

.. image:: ../../../images/gpl3.png
    :target: https://www.gnu.org/licenses/gpl-3.0.en.html

Consider joining the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__, to speak with other developers about this topic under the HAL-DEV, #Linux channel.

Step 1 - getting started
========================

- Determine which microprocessor the new flight controllers uses. If it is used in a board we already support, for example BCM283x used in Raspberry Pi or TI Sitara/OMAP used in BeagleBones, then the port should be relatively straight forward.
- Choose a new board name.
- Reserve a new HAL_BOARD_SUBTYPE_LINUX_$(NEW_BOARD_NAME) in libraries/AP_HAL/AP_HAL_Boards.h
- Ensure the spidev and i2cdev kernel modules are built if ArduPilot will be using them.

.. tip:: Choose your board name carefully! Use 13 characters or less for your board name, otherwise it may be truncated when the board name is sent from the flight controller to a ground station such as Mission Planner.

Step 2 - Toolchain Configuration
================================

In Tools/ardupilotwaf/boards.py:

- Create a new class and select the appropriate toolchain, usually arm-linux-gnueabihf. For an example see "class pxfmini(linux) as an example"
- Update CONFIG_HAL_BOARD_SUBTYPE with the newly reserved board subtype.

In Tools/scripts/board_list.py:

- Add new board name to self.boards[] list.

Step 3 - configure and build examples and firmware for the board
================================================================

Follow the :ref:`Building the code <building-the-code>` instructions or take a shortcut and read the `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ file which includes doing the following:

- ``cd ardupilot`` (or wherever you have :ref:`cloned <git-clone>` ArduPilot to)
- ``./waf configure --board new-board``
- ``./waf copter``

Work through any issues highlighted by '#error' messages in the codebase, which will require per-board configuration.

This is also a good time to build any relevant :ref:`Library Example Sketches <learning-ardupilot-the-example-sketches>`.

Step 4 - upload and run the examples and firmware
=================================================

If building natively, executables are already loaded on the target system, and can be executed directly.

If using waf to upload (Linux, MacOSX only), see configuration commands in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ 
to set up waf's --upload flag. Other options for cross-compiled builds include SCP or NFS.

See :ref:`Startup options <ardupilot-on-linux-starting>` for command-line options, configuring at least one port for MavLink, and launching ArduPilot.

Step 5 - peripheral functions and drivers
=========================================

Many boards will use the Linux userspace APIs for I2C, UART, and SPI and only need configuration and associated kernel modules.
See your CPU, BSP, and board's documentation for pin muxing and device tree configuration. The following peripherals tend to be more complicated.

SPI Support
-----------
To configure SPI-based devices, the Linux userspace SPI APIs are used, needing only the per-board configuration(bus ID, chip select ID, mode, rates) in SPIDeviceManager::_device[] in libraries/AP_HAL_Linux/SPIDevice.cpp.

GPIO Support
------------
Most ArduPilot Linux boards use one of two methods for GPIO access: sysfs or mmap.

For sysfs:

- GPIO access through sysfs is deprecated.
- Create new libraries/AP_HAL_Linux/GPIO_$(NEW_BOARD_NAME).cpp and .h, filled out with the needed _$(NEW_BOARD_NAME)_GPIO_MAX and pin_table.

For libgpiod(Todo):

- port ardupilot to support a libgpiod backend of pinMode(), read(), write().
- Implement the per-board configuration

For mmap():

- See GPIO_BBB.cpp and GPIO_RPI.cpp as examples
- Create new libraries/AP_HAL_Linux/GPIO_$(NEW_BOARD_NAME).cpp and .h
- Locate the memory file, either /dev/mem or /dev/gpiomem, and open() it.
- Locate the offset of the GPIO memory region in the file. If using /dev/mem, check the processor reference manual or Linux device-tree files. For /dev/gpiomem, use zero.
- mmap() the GPIO registers into ArduPilot using the above offset.
- Implement pinMode(), read(), write() functions using the memory mapped GPIO registers.

.. tip:: CPUs with multiple GPIOs per bank with only a DATA or OUT register without SET and CLEAR registers may need to avoid the mmap backend, as read-modify-write accesses to a GPIO DATA register may not be threadsafe when the kernel or other userspace programs use GPIOs in the same bank.

For either approach, edit libraries/AP_HAL_Linux/GPIO.h to include the appropriate GPIO_$(NEW_BOARD_NAME) header for the board.

RCInput
-------
Several different RCInput mechanisms are supported by different boards, including PPM, SBUS, UART, and UDP.

PPM input is handled per-CPU, usually with either a DMA or utility microcontroller like the PRU on TI chips.

Examine the libraries/AP_HAL_Linux/RCInput_* files to see if a suitable one exists, or create one.

Configure rcinDriver in libraries/AP_HAL_Linux/HAL_Linux_Class.cpp using CONFIG_HAL_BOARD_SUBTYPE.

RCOutput
--------
Similar to RCInput, examine the libraries/AP_HAL_Linux/RCOutput_* files to see if a suitable one exists, or create one.

For CPUs without corresponding hardware to generate RCOutput signals, additional hardware may be used, like NXP's PCA9685 for PWM generation.

Configure the rcoutDriver in libraries/AP_HAL_Linux/HAL_Linux_Class.cpp using CONFIG_HAL_BOARD_SUBTYPE.

Step 7 - bring up the sensors
=============================

Add sensor related configuration to libraries/AP_HAL/board/linux.h.

Start with the baro first, then IMU, then compass and finally any other sensors
the default sensor orientation should also be filled in along with other things.

Upload the examples and firmware and test the sensors are working.

Step 8 - configure parameter, log, and terrain storage
======================================================

Configure HAL_BOARD_LOG_DIRECTORY, HAL_BOARD_TERRAIN_DIRECTORY, HAL_BOARD_STORAGE_DIRECTORY in libraries/AP_HAL/board/linux.h.

Next Steps
==========

If you have gotten this far, congratulations you have ported ArduPilot to a new board!  Please reach out to the other developers on the `ArduPilot Discord Chat <https://ardupilot.org/discord>`__ to announce your success.
