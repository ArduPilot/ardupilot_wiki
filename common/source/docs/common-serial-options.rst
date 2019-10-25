.. _common-serial-options:

============================
Serial Port Hardware Options
============================

This page describes the hardware configuration options for the serial ports. Currently, these options are supported only on specific flight controllers.


SERIALx_OPTIONS Parameter
=========================

Every serial port has in addition, to its baud rate (``SERIALx_BAUD``) and protocol format (``SERIALx_PROTOCOL``), the ability to invert its RX input and/or TX data, operate in half-duplex mode, and/or swap its RX and TX inputs.

For example, for direct connection to FRSky SPort telemetry, normally inverters and diode OR externally would be required. With SERIALx_OPTIONS bitmask set to 7, direct connection to the SPort can be accomplished from a serial port.

Bitmask Options
---------------

- if bit 0 is set, then RX data received is inverted internally.
- if bit 1 is set, the TX data is inverted before outputting.
- if bit 2 is set, then HalfDuplex operation using the TX pin is implemented.
- if bit 3 is set, then the TX and RX pins are effectively swapped internally.

.. note:: HalfDuplex is supported on all ChiBiOS based flight controllers, but all other options are only supported on boards with F7 or H7 microprocessors.