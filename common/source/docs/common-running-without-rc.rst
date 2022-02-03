.. _common-running-without-rc:

========================================
Running ArduPilot without an RC Receiver
========================================

This article provides an overview of how ArduPilot can be configured
to safely run with no RC receiver attached.

.. note::

   Fitting your vehicle with an RC receiver allows for additional recovery measures should your vehicle misbehave.  Particularly while creating and developing a vehicle we strongly recommend flying with an RC receiver/transmitter pair providing this additional recovery mechanism.

  Before making the decision to configure your vehicle without an RC receiver, consider options such as ignoring RC failure for failsafe purposes; see "Configuring for Emergency RC Receiver Use", below.

Configuring for no RC Receiver
==============================

Configuring Failsafes
---------------------
 - this is somewhat vehicle-specific, but for ArduCopter FS_THR_ENABLE should be zero THR_FS_ENABLE should be zero
 - it is strongly suggested that a GCS failsafe be configured in the case that an RC failsafe is not configured.


Configuring for RC input to be ignored
--------------------------------------

There have been instances where EMI has created the illusion that PPM-SUM has been input to the autopilot where no RC receiver is in fact present.  This can be guarded against by:

 - as stated on :ref:`the page about RC_OPTIONS <common-rc-options>`, setting bit 0 to 1 will disable RC input to the vehicle
 - reducing the protocols that ArduPilot will decode to just the protocols which will be emitted by the RC receiver using the :ref:`RC_PROTOCOLS<RC_PROTOCOLS>` parameter


Configuring for Partial RC Receiver Use
---------------------------------------

 - narrow options using RC_PROTOCOLS
 - ignore RC failsafes when in auto mode but not in other modes
 - set up GCS failsafes
 - switch positions (workflows)
