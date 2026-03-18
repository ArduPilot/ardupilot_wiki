.. _security-landing-page:

========
Security
========

This page describes how to protect an ArduPilot from external threats.

.. toctree::
    :maxdepth: 1

    Secure Firmware (tamper-proof) <secure-firmware>

- :ref:`MAVLink2 Signing <common-MAVLink2-signing>`

Security Attack Surface
-----------------------

ArduPilot works in a resource-constrained environment.  We can't afford all of the sanity checks that we might otherwise include.

To this end, we do not consider every input to the autopilot firmware potentially malicious.  We trust our SPI-connected inertial sensors to be well-behaved, for example.

Ground Control Stations
.......................

Most notable amongst "trusted" data sources are connections to the Ground Control Station.  We expect GCSs to be well-behaved in terms of the data sent to the autopilot.  Remember that your GCS can disarm your vehicle mid-air or force it into terrain as a matter of course.

We disable floating point exceptions in the embedded firmware, meaning that a lot of floating point operations which would result in a Floating Point Exception now simply don't.  By default we do NOT disable floating point exceptions in SITL, allowing errors in Ground Control Stations to be picked up in SITL rather than when someone is flying a real vehicle!

One exception to the trusted-ground-control-station model is if :ref:`MAVLink Signing <common-MAVLink2-signing>` is enabled.  If data coming into ArduPilot on a serial port configured for signed-only MAVLink2 connections from a GCS which does not have the signing key can cause the vehicle to misbehave (ignoring DOS attacks), we *do* consider this to be a security issue.

Disabling FPE in SITL
.....................

To more-closely approximate what happens on our embedded platforms, you can use the ``SIM_FLOAT_EXCEPT`` parameter to disable floating point exceptions in ArduPilot SITL.  This may help find real problems with trying to use fuzzers to find problems with the ArduPilot codebase.
