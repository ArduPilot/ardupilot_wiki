.. _common-multiple-rx:

================================
Multiple Radio Control Receivers
================================

It is possible to use multiple radio control receivers in ArduPilot for redundancy or hand-off between multiple transmitters.

Configuration
=============

.. note:: This configuration only is valid for autopilots that use an IOMCU co-processor. Check the product description to be sure that the autopilot includes either an STMF100 or STMF103 co-processor. Usually these outputs have servo/motor outputs labeled as MAIN (the co-processor) and AUX (from the main cpu).

The additional receivers can be attached to any Serial Port's RX input as long as they use a serial protocol.
These are: SBUS, FPort, DSM, DSM2, DSM-X, IBus, SUMD, and SRXL (and CRSF if the TX output of the UART is also used).

For whichever Serial Port the receiver is attached, the ``SERIALx_PROTOCOL`` should be set to 23. The baud rate is ignored and auto-detected, as well as the type of the receiver.

.. note:: SBUS and FPort must either be externally inverted or the SERIALx_OPTIONS be set for inversion (F and H processor types only are capable of this).

The :ref:`RC_OPTIONS<RC_OPTIONS>` bit 10 must be set.

.. note: only one UART can be designated to use protocol "23". Using the normal RC input..

Failsafe and Changeover
=======================

The autopilot will always use the receiver attached to the IOMCU's RCin, if providing valid RC signals. If that receiver fails to provide valid RC signals, it will then switch to the other receiver if it is outputting valid RC signals. It will reverted to the first if it restarts outputting valid RC signals.

Note that the previously calibrated ``RCx_MAX`` , ``RCx_MIN``, ``RCx_REVERSED``, ``RCx_DZ`` and ``RCx_TRIM`` parameters for each channel will be used by the active receiver and could vary between receivers, especially if different types are mixed. It is recommended that all receivers be of the same manufacture and type.

.. note:: lowering throttle to below :ref:`FS_THR_VALUE<FS_THR_VALUE>`  on the first receiver will not force a changeover, but rather force a normal radio failsafe. Only loss of RC signals from the first receiver will cause a changeover.

If all receivers become inactive (no pulses), then the normal radio failsafe handling will begin.

.. warning:: It is important that all receivers be configured for no pulses for radio loss failsafe for the changeovers to work properly.

Inversion
=========

Some protocols, like SBus, require that the signal be inverted . Autopilots using F7/H7 cpus have the ability to invert the RX input via its ``SERIALx_OPTIONS`` bitmask. Otherwise an external inverter must be used between the receiver data output and the Serial Port's RX input.

Receivers Having Telemetry/Video Transmitters Integrated
========================================================

Some receivers have telemetry and/or video transmitters integrated. Having multiple units, operating on the same band, in the same vehicle, can cause reduction of range in both units, even if using spread-spectrum techniques. CRSF, FrSky, DragonLink, etc. are some examples of these types of receivers. Having the antennas as widely separated as possible will help, but not eliminate some range reduction. Best case is to have the units operating in different radio bands.

Be sure to do range checks on the ground using the transmitter's range check mode before flight testing.
