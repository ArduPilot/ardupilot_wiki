.. _common-erdi-rangefinder:

=================================
ERDI TECH Lidar (VB and H series)
=================================

.. note::
    Support for these sensors is available in firmware versions 4.8 and later, on autopilots with more than 1MB of flash.

`ERDI TECH <https://erdilrf.com/models/>`__ (ERDILRF) makes small single-point laser rangefinder modules with a UART output (905 nm, except the LRF10VB which uses 940 nm). ArduPilot supports the VB and H series, which share one 4-byte serial frame but report distance in different units, so each series has its own :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` value.

========  ===========================================  ==============  ==============  ==================
Series    Models (published range)                     Supply voltage  Distance unit   ``RNGFND1_TYPE``
========  ===========================================  ==============  ==============  ==================
VB        LRF10VB (9m)                                 4.5 - 5.5V      millimetres     49 (ERDI-VB-Serial)
VB        LRF22VB (20m)                                3 - 3.6V        millimetres     49 (ERDI-VB-Serial)
VB        LRF50VB (50m)                                3.3 - 5V        millimetres     49 (ERDI-VB-Serial)
H         LRF50H (50m), LRF100H (100m)                 9 - 36V         centimetres     50 (ERDI-H-Serial)
H         LRF200H (200m), LRF300H (300m),              9 - 36V         centimetres     50 (ERDI-H-Serial)
          LRF600H (600m)
========  ===========================================  ==============  ==============  ==================

The published range is for a 90% reflectivity target. The range for a darker target is shorter, see each model's page and datasheet linked below. The LRF1500H, which uses a different (5-byte) frame, is not supported by this driver.

Where to Buy
============

ERDI TECH sells directly from its `website <https://erdilrf.com/models/>`__.

Connecting to the Autopilot
===========================

Check the model's datasheet for the connector, pin order and supply voltage before powering it. The supply voltage differs between models (see the table above): for example the LRF22VB must be powered from 3 - 3.6V, while the H series needs 9 - 36V and cannot be powered from the autopilot's 5V serial port pin.

Connect the sensor's TX to the RX of any spare autopilot Serial/UART port and the sensor's RX to that port's TX (TX and RX are crossed), and connect the grounds. The example parameters below assume SERIAL4 is used.

Set the following parameters:

-  :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` = 9 (Rangefinder)
-  :ref:`RNGFND1_TYPE <RNGFND1_TYPE>` = 49 (ERDI-VB-Serial) for the VB series, or 50 (ERDI-H-Serial) for the H series. Reboot after setting this.
-  :ref:`RNGFND1_MIN <RNGFND1_MIN>` = 0.05
-  :ref:`RNGFND1_MAX <RNGFND1_MAX>` = the range you have tested for the model and target, up to its published range. For the LRF22VB and LRF50VB set it slightly below the rated range (for example 19.9 for the LRF22VB). These two models send their rated maximum distance, rather than an out-of-range code, when there is no valid return, so a value just below the rated range lets ArduPilot treat that reading as out of range.
-  :ref:`RNGFND1_ORIENT <RNGFND1_ORIENT>` = 25 (Down) if the sensor faces downward

The driver sets the port's baud rate to 460800 (the modules' default) itself, so :ref:`SERIAL4_BAUD<SERIAL4_BAUD>` does not need to be changed.

Notes
=====

- The H series streams distance at up to 1kHz from power-up. The driver averages the valid readings received in each update.
- The VB series accepts start and stop ranging commands. If no valid reading has been received for 1 second, the driver sends the "Start Ranging" command once a second until readings resume.
- Readings greater than :ref:`RNGFND1_MAX <RNGFND1_MAX>` are treated as out of range rather than being averaged in.

Testing the sensor
==================

Distances read by the sensor can be seen in the Mission Planner's Flight
Data screen's Status tab. Look closely for "rangefinder1".

More information
================

- `LRF22VB <https://erdilrf.com/models/lrf22vb/>`__ and `LRF50H <https://erdilrf.com/models/lrf50h/>`__ model pages (the other models are linked from the `model list <https://erdilrf.com/models/>`__)
- `Serial protocol and model tables <https://github.com/ErdiTech/erdi-lrf-drivers>`__
