.. _interlink-emulation:

====================================================
Interlink Simulator Controller Emulation with OpenTX
====================================================

It is possible to closely emulate the Interlink DX or Elite with an OpenTX transmitter in RealFlight. This allows using your existing OpenTX transmitter if RealFlight software only is purchased. Since the ArduPilot SITL models for use with RealFlight assume that an Interlink DX is being used, setting up your OpenTX transmitter to exactly emulate it prevents having to change any transmitter settings for its use.

Interlink DX Mapping
====================

The Interlink DX controller is automatically mapped as below when used with RealFlight:

+--------------------+-------------------------+-------------------+
+Interlink DX control| InterLink Elite control | RC channel output +
+====================+=========================+===================+
+  Aileron           |   Aileron               |        1          +
+--------------------+-------------------------+-------------------+
+  Elevator          |   Elevator              |        2          +
+--------------------+-------------------------+-------------------+
+  Throttle          |   Throttle              |        3          +
+--------------------+-------------------------+-------------------+
+  Rudder            |   Rudder                |        4          +
+--------------------+-------------------------+-------------------+
+  Switch C          |   Ch 5 (DualRates)      |        5          +
+--------------------+-------------------------+-------------------+
+  Switch D          |   Ch 6 (Flaps)          |        6          +
+--------------------+-------------------------+-------------------+
+  Switch A          |   Ch 7 (Smoke)          |        7          +
+--------------------+-------------------------+-------------------+
+  Switch B          |   Ch 8 (Mode)           | 8 (always mode ch)+
+--------------------+-------------------------+-------------------+
+  Switch H          |                         |       9           +
+--------------------+-------------------------+-------------------+
+  Switch F          |                         |       10          +
+--------------------+-------------------------+-------------------+
+  Switch G          |                         |       11          +
+--------------------+-------------------------+-------------------+
+  Top Button        |                         |       12          +
+--------------------+-------------------------+-------------------+
+  Knob              |                         |       13          +
+--------------------+-------------------------+-------------------+
+  Left Slider       |                         |       14          +
+--------------------+-------------------------+-------------------+
+  Right Slider      |                         |       15          +
+--------------------+-------------------------+-------------------+


OpenTX Emulation
================

Set up a model in the Transmitter as a model named SIM and program the mixes as below using 100% throws/gains, and no offsets, curves, etc depending on which Interlink controller you wish to mimic. Turn off the internal RF section to conserve power.


These most closely mimic the Interlink controllers' physical layout. Of course, you can map any OpenTX control/switch to any channel you desire, if you wish a different layout than the Interlink controller.

+-----------------+--------------------+
+OpenTX control   |  RC channel output +
+                 |   (DX / Elite)     +
+=================+====================+
+  Aileron        |        1/1         +
+-----------------+--------------------+
+  Elevator       |        2/2         +
+-----------------+--------------------+
+  Throttle       |        3/3         +
+-----------------+--------------------+
+  Rudder         |        4/4         +
+-----------------+--------------------+
+  SA             |        5/5         +
+-----------------+--------------------+
+  SB             |        6/8         +
+-----------------+--------------------+
+  SF             |        7/-         +
+-----------------+--------------------+
+  SE  (Note 1)   |        8/-         +
+-----------------+--------------------+
+  SH             |        9/-         +
+-----------------+--------------------+
+  SD             |        10/-        +
+-----------------+--------------------+
+  SG  (Note 1)   |        11/7        +
+-----------------+--------------------+
+  SC             |        12/-        +
+-----------------+--------------------+
+  S2             |        13/6        +
+-----------------+--------------------+
+  Left Slider    |        14/-        +
+-----------------+--------------------+
+  Right slider   |        15/-        +
+-----------------+--------------------+

Notes:

#. If you are using a QX7, then you will need to use S1 instead of SE as the mode channel (8) if emulating an Interlink DX. Since there is no SG switch, you will not have channel 11.


.. note:: Currently, when using any controller with SITL flightaxis in RealFlight, only the first 8 channels are passed to the SITL (soon will be upgraded to the first 12), but higher channels can be still used in normal RealFlight simulations, if the model uses them.


Pre-configured OpenTX .otx files for Taranis 9D, QX7, and Horus style transmitters are available `here <https://github.com/ArduPilot/ardupilot_wiki/tree/master/scripts/OpenTX_configs>`_.
