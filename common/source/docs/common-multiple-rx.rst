.. _common-multiple-rx:

================================
Multiple Radio Control Receivers
================================

It is possible to use multiple radio control receivers in ArduPilot for redundancy or hand-off between multiple transmitters.

Configuration
=============

The additional receivers can be attached to any Serial Port's RX input as long as they use a serial protocol.
These are: SBus, DSM, DSM2, DSM-X, IBus, SUMD, and SRXL.

For whichever Serial Port the receiver is attached, the ``SERIALx_PROTOCOL`` should be set to 23. The baud rate is ignored and auto-detected, as well as the type of the receiver.

Failsafe and Changeover
=======================

The autopilot will use the first active receiver it finds, beginning with the normally connected receiver. If pulses from that receiver fail, it will then switch to the next active receiver and will use it even if the prior receiver becomes active again.

Note that the previously calibrated ``RCx_MAX`` , ``RCx_MIN``, ``RCx_REVERSED``, ``RCx_DZ`` and ``RCx_TRIM`` parameters for each channel will be used by the active receiver and could vary between receivers, especially if different types are mixed. It is recommended that all receivers be of the same manufacture and type.

If all receivers become inactive (no pulses), then the normal radio failsafe handling will begin.

.. warning:: It is important that all receivers be configured for no pulses for radio loss failsafe for the changeovers to work properly.

Inversion
=========

Some protocols, like SBus, require that the signal be inverted . Autopilots using F7/H7 cpus have the ability to invert the RX input via its ``SERIALx_OPTIONS`` bitmask. Otherwise an external inverter must be used between the receiver data output and the Serial Port's RX input.
