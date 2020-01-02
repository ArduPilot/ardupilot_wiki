.. _testing-with-replay:

===================
Testing with Replay
===================

Introduction
============

Replay is a program that takes a dataflash log file and replays it through the latest master code allowing a sort of simulation based on real data.  
This can be useful when trying to recreate the exact situation which produces a bug or to test EKF tuning parameters to see how they would have performed in the same situation. 
Replay only runs on Linux/Ubuntu and only using dataflash logs from a high speed CPU such as the PX4/Pixhawk running a version of Copter/Plane/Rover from May 2014 or later (i.e. AC3.2-dev or higher).

It is recommended that if a problem is reproducible that the dataflash log be generated with both the `LOG_REPLAY` and `LOG_DISARMED` parameters set to 1.

.. image:: ../images/Replay_EKFVsINAV.png
    :target: ../_images/Replay_EKFVsINAV.png

Dataflash log messages required for Replay
==========================================

If `LOG_REPLAY` is not set, the following dataflash messages must be enabled: AHRS2, BARO, EKF1, EKF2, EKF3, EKF4, GPS, IMU, IMU2, MAG, MAG2.

Building Replay
===============

On your Linux or Ubuntu machine, from the root directory of an ArduPilot repository:

.. code-block:: bash

    ./waf configure --board=linux
    ./waf build --target=tools/Replay

.. note::

    You may need to install `sudo apt-get install pkg-config`

This will create a file called ``build/linux/tools/Replay``.

Using Replay
============

Display the Replay help instructions:

.. code-block:: bash

    build/linux/tools/Replay -- --help

Run a log through Replay to generate the plot and EKF data files:

.. code-block:: bash

    build/linux/tools/Replay -- MyLogFile.BIN

.. note::

    You may need to explicitly set the loop rate with ``-r400`` and ignore floating point errors with "--no-fpe"

This will produce an output file 1.BIN in the folder "./logs"

Use mavgraph.py to graph the data

.. code-block:: bash

    mavgraph.py MyLogFile.BIN GPA.SAcc NKF4[0].SV


.. image:: ../images/Replay_GPSSAcc_NKF4SV.png
    :target: ../_images/Replay_GPSSAcc_NKF4SV.png

This example graphs the changes in speed innovation of the EKF against the GPS speed measurement accuracy

Changing parameters
===================

Simulation parameters may be changed before replaying a log using the option: ``-parm NAME=VALUE`` (this sets the parameter ``NAME`` to
``VALUE``). 
The parameters which may be edited are those listed by running the `:ref:`param show`` command in `SITL <setting-up-sitl-on-linux>`.

For example, to change the EKF I gate value to 1,000, run the command:

.. code-block:: bash

    build/linux/tools/Replay -- --no-fpe --parm EK2_VEL_I_GATE=1000 log_1.bin
