.. _balance_bot-hardware: 

================
Hardware Options
================

This page lists the hardware required for an autonomous Balance Bot

Frame
=====

Any readily available or custom made frame suitable for Balance Bots can be used. Ensure that there is sufficient space to hold all the components.

For a 3d printable frame, check out :ref:`Arduroller<reference-frames-arduroller>`.

Autopilot
=========

The autopilot needs at least two PWM outputs for controlling the two motors and four :ref:`GPIO pins<common-gpios>` to connect to two :ref:`wheel encoders <wheel-encoder>`.  Some autopilots without an IOMCU will not have enough GPIOs to support wheel encoders.

Motors and Wheel Encoders
=========================

The recommended solution is to use brushed motors with built-in quadrature :ref:`wheel encoders <wheel-encoder>` but other motors may be used if external encoders are attached to the motor shafts.  Note that only quadrature encoders with two output pins are supported.  :ref:`Wheel encoders <wheel-encoder>` allow faster and more precise control of each wheel which greatly improves the vehicle's balance.

Stepper Motors and motors that use UART/I2C interface are not currently supported.

Gear backlash is a problem with many geared dc motors and this can negatively affect stability. It is recommended to use motors that are specified to be zero-backlash or have very little backlash.

ESCs/Motor Drivers
==================

A list of verified ESCs/motor drivers can be fouind on the :ref:`brushed motors<common-brushed-motors>` page.

GPS + Compass
=============

A GPS+Compass module is recommended if the balance bot will be used outdoors in autonomous modes (e.g. Auto, Guided, RTL).  A compass is required so the vehicle can estimate its heading.  For more details on supported GPS hardware, refer to the :ref:`GPS/Compass page<common-positioning-landing-page>`.

Telemetry Radio(optional)
=========================

Telemetry radios are very useful for tuning and debugging. For more information on supported hardware options refer :ref:`telemetry<common-telemetry-landingpage>` page.

