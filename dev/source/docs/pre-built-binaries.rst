
.. _pre-built-binaries:

============================
ArduPilot Pre-Built Binaries
============================

The ArduPilot firmware server at https://firmware.ardupilot.org has pre-built binaries for many common flight boards.

Build Categories
----------------

For each vehicle type there are 4 categories of pre-built binary:

- stable: the most recent stable release, recommended for normal usage
- beta: the most recent beta release, this is for users who wish to help test new features before they go into a stable release
- latest: the most recent build of our git master branch, this contains the latest code. Use with caution, this version has undergone very little testing
- historical builds: These are older version of the latest builds. 
  These can be used to find when a bug was introduced by trying a range of build dates

File Types
----------

Within the build directories you will find files of the following
types:

 - ``*.apj`` files: these are "ArduPilot JSON" firmwares, which contain a firmware that can be loaded by ArduPilot compatible ground station software
 - ``*.px4`` files: these are an older name for an apj file, and use the same format
 - ``*.hex`` files: these are firmwares in Intel hex format, for loading with DFU loading tools. 
   These are used for boards which don't come with an ArduPilot compatible bootloader
 - ``*_with_bl.hex`` files: these are variants of the hex files with the bootloader built-in. 
   They can be used to install both the bootloader and ArduPilot vehicle firmware in one step using a DFU loading tool

Build Variants
--------------

For some types of flight boards multiple builds in separate directories are provided. 
For example, you can find a firmware suitable for a Pixhawk2.1 Cube in both the px4-v3 directory and the CubeBlack directory. 
These variants use different underlying RTOS code (NuttX and ChibiOS). 
As we move the project away from NuttX to ChibiOS the NuttX builds will be removed over time.
