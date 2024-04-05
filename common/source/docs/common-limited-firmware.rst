.. _common-limited_firmware:

==========================================
Firmware Limitations on AutoPilot Hardware
==========================================

The ArduPilot firmware in some configurations exceeds 1 MB in size. Some autopilots may not have enough flash memory to store the full firmware.

For the affected autopilots, a reduced firmware is generated. This firmware omits less-commonly used features in order to reduce the firmware size to less than 1 MB.

:ref:`The missing features are listed on this page<binary-features>` for each board type for the current "latest" firmware. If you require any of these features, you can try to create a build with them in it (at the expense of other non-needed features) using the `Custom Firmware Build Server <https://custom.ardupilot.org>`__. The missing features list for a board are listed in the same groups and labels as those shown on the Custom Firmware Build Server.

.. note:: Currently the above page of missing features is auto-generated and has a bug, While this is being investigated and corrected, the definitive list for a given board and firmware version can be found in its firmware folder on `the firmware server <https://firmware.ardupilot.org/>`__ in its "features.txt' file.

Using the server does require you to know which features you DO want to have and check them, as well as the ones desired that are currently missing.
A table of all current build options that can be selected are shown :ref:`here<all-features>`.

.. _ram_limitations:

RAM Limitations
===============

There may be insufficient RAM available in some autopilots to support all enabled firmware features. Some possible symptoms are:

- MAVFTP does not work. Parameter downloads to GCS are delayed starting while the GCS tries to establish a MAVFTP link which cannot be setup, and then proceed slowly to download parameters using the normal download method.
- Compass Calibration will not start
- Logging will not start
- Terrain downloading from GCS will not start
- Interfaces will not initialize, such as CAN: "Failed to initialise CAN Interface xx"

If this occurs, several possible options are available to allow temporary use of MAVFTP, download terrain tiles, allow a driver to be loaded, and/or Compass Calibration. All require a reboot to take effect:

- Make sure IMU Batch Sampling (used for FFT analysis) is not running by setting :ref:`INS_LOG_BAT_MASK<INS_LOG_BAT_MASK>` = 0.
- Try temporarily disabling logging by setting :ref:`LOG_BACKEND_TYPE<LOG_BACKEND_TYPE>` to 0, then returning to 1 (default) after calibrating.
- Reduce the size of :ref:`LOG_FILE_BUFSIZE<LOG_FILE_BUFSIZE>`. However, lowering below 16KB can introduce small gaps in the log. This may be used temporarily to download terrain or calibrate compass.
- Disable Terrain Following temporarily by setting :ref:`TERRAIN_ENABLE<TERRAIN_ENABLE>` to 0.
- Disable SmartRTL on Copter by setting :ref:`SRTL_POINTS<SRTL_POINTS>` = 0.
- If using DroneCAN, try reducing the memory allocation from the default of 16KB (for two nodes) using the ``CAN_Dx_UC_POOL`` parameters.

.. note:: in Mission Planner's STATUS tab, you can monitor the "freemem" status for current free RAM. Be aware, that its a total of unallocated memory and that everything that requires a memory allocation needs it to be one, contiguous block. But the status will give an indication of what is available. For example, MAVftp needs ~ 12K contiguous block to start.

.. toctree::
    :hidden:

    Features <binary-features>

[copywiki destination="plane,copter,rover,blimp"]
