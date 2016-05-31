.. _quadplane-overview:

QuadPlane Overview
==================

A QuadPlane is a combined fixed wing and MultiCopter aircraft. This sort
of aircraft brings the benefit of vertical takeoff and landing,
significantly greater speed and range of travel, and the ability to
hover and perform copter-like tasks at the destination.

QuadPlane is built upon Plane, but adds control over between 4 and 8
horizontal rotors. Additional modes and commands allow a QuadPlane to
take off, land and fly like a copter, and to smoothly transition
between the Plane and Copter-like modes in both automatic and
autopilot-assisted modes. The additional rotors can also provide lift
and stability in normal Plane modes.

Installing the Firmware
=======================

QuadPlane support is in APM:Plane releases from 3.5.0 onwards. The
normal instructions for installing the Plane firmware apply.

When you install the plane firmware and look in the parameter list you
will see a Q_ENABLE parameter. That ddefaults to zero, which disables
QuadPlane support. Setting Q_ENABLE to 1 will enable QuadPlane
support. You then need to refresh your parameter list to see all the
other QuadPlane options. All QuadPlane specific parameters start with
Q_.
