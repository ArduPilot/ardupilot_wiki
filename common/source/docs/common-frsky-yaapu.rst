.. _common-frsky-yaapu:

=======================================
Yaapu FrSky Telemetry Script for OpenTX
=======================================

This is an open source LUA script to display :ref:`FrSky passthrough telemetry <common-frsky-passthrough>` on Horus X10,X12 and Taranis X9D,X9E and QX7 radios. Support for dual flight batteries is also provided. Hardware details are shown in  :ref:`FrSky passthrough telemetry <common-frsky-passthrough>` also.


Display on Taranis

.. image:: ../../../images/x9d.png
    :target: ../_images/x9d.png

Display on Horus

.. image:: ../../../images/x10.png
    :target: ../_images/x10.png

Display on QX7

.. image:: ../../../images/x7.png
    :target: ../_images/x7.png


Details can be found `here <https://discuss.ardupilot.org/t/an-open-source-frsky-telemetry-script-for-the-horus-x10-x12-and-taranis-x9d-x9e-and-qx7-radios/26443>`__. Latest script releases are `here <https://github.com/yaapu/FrskyTelemetryScript/releases>`__.

The script is also compatible with the excellent `MavlinkToPassthru converter firmware <https://github.com/zs6buj/MavlinkToPassthru>`__ by Eric Stockenstrom which allows alternative telemetry transport methods, such as the data modem in DragonlinkRC RC systems, to feed MAVLink data into these transmitters for display by this LUA script.

Requires `OpenTX 2.2.x <http://www.open-tx.org/>`__ (2.2.3 recommended) and a recent release of ArduPilot with support for :ref:`common-frsky-passthrough` .






