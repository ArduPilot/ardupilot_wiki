.. _balance_bot-hardware: 

================
Hardware Options
================
This page lists the hardware options that can meet all of the Balance Bot specific requirements. Though there is no limitation to running Balance Bot on any Rover compatible hardware, not all features and drive modes may be supported in certain configurations.

Autopilot Board
===============
Autopilot Boards than can be used with Balance Bot are limited by the number of PWM and aux pins that are available. Currently supported boards are:

#. **Pixhawk:**  All full sized versions
#. **Pixracer:**  When using pixracer, only :ref:`RC PWM controlled<common-brushed-motors>` or :ref:`Brushed BiPolar<common-brushed-motors>` motor drivers can be used. Four pins on the pixracer will be used for wheel encoders leaving only two for driving motors.

.. warning:: Miniature versions of Pixhawk, like pixhawk mini, pixfalcon and Openpilot/Betaflight boards cannot interface wheel encoders

Balance Bot Frame
=================
Any readily available or custom made frame suitable for Balance Bots can be used. Ensure that there is sufficient space to hold all the components.

For a 3d printable frame, check out :ref:`Arduroller<reference-frames-arduroller>`.

Motors with encoders
====================
Brushed Motors with quadrature encoders(two pin output) are recommended for Balance Bots. There is no restriction on using brushless motors, additional quadrature encoders will have to be added to run any mode other than Hold and Manual. Stepper Motors and motors that use UART/I2C interface are not currently supported.

Gear backlash is a problem with many geared dc motors and this can badly affect stability of Balance Bots. It is recommended to use motors that are specified to be zero-backlash or have very less backlash.

Motor Drivers/ESC
=================
For a full list of ArduPilot supported brushed motor drives/ESCs, refer the :ref:`brushed motors<common-brushed-motors>` page.

Wheel Encoders
==============
When using motors without an an inbuilt encoder, external encoders that can be attached to the motor shaft, can be used. Only quadrature encoders with two output pins are currently supported. For more information, refer the :ref:`wheel encoder<wheel-encoder>` page.


GPS + Compass(optional)
=========================
A GPS+Compass module is required for running Auto, Guided and RTL Modes. Using compass is also recommended for steering in Acro Mode. For more details on supported GPS hardware, refer :ref:`GPS<common-positioning-landing-page>` page.

Telemetry Radio(optional)
=========================
Telemetry radios are very useful for tuning and debugging. For more information on supported hardware options refer :ref:`telemetry<common-telemetry-landingpage>` page.








