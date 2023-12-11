.. _traditional-helicopter-tuning:

===============================
Traditional Helicopter – Tuning
===============================

This guide is broken out for manual and AutoTune tuning techniques.  Both tuning techniques will require the user to perform the initial setup steps.  These initial setup steps are very important and have a significant impact on the quality of the tune.  Both manual and AutoTune cover the tuning of the feedforward gain, Rate PID gains, and angle P gain.  Additional tuning topics cover adjusting the aircraft feel, further fine tuning of the I gain as well as integrator management.

.. warning:: For making setting changes to traditional helicopters, users are reminded to use only the Full or Complete Parameter List, or the Heli SETUP page in your ground station software. **Do not use the Basic, Extended or Advanced Tuning pages that are designed for multi-rotor aircraft in Mission Planner.** These pages will make unwanted setting changes to traditional helicopters. And remember to write the changes to the autopilot after making them or they won't be saved!

.. note:: If the reader is unfamiliar with PID control systems then reading :ref:`traditional-helicopter-control-system` is recommended.


.. toctree::
    :maxdepth: 1

    Arducopter Control System Description <traditional-helicopter-control-system>
    Preparing for Tuning<traditional-helicopter-tuning-preparing>
    Manual Tuning Wiki <traditional-helicopter-manual-tuning>
    AutoTune Wiki <traditional-helicopter-autotune>
    Additional Tuning Topics <traditional-helicopter-tuning-other-topics>
