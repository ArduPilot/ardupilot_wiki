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

    You may need to explicitly set the loop rate with ``-r400``

This will produce six output files: plot.dat, plot2.dat, EKF1.dat, EKF2.dat, EKF3.dat, EKF4.dat

Look at the raw data to see which values are available to be plotted:

``less plot.dat`` (you can replace ``plot.dat`` with any of the other
six files produced)

.. image:: ../images/Replay_PlotDatColumns.png
    :target: ../_images/Replay_PlotDatColumns.png

Use the simple plotit.sh script to graph some data. 
Below is the command to compare the EKF calculated altitude with the older Inertial Nav calculated altitude and the flight's actual altitude.

.. code-block:: bash

    ./Tools/Replay/plotit.sh EKF.Alt INAV.Alt FLIGHT.Alt

.. image:: ../images/Replay_EKFInavFlightAlt.png
    :target: ../_images/Replay_EKFInavFlightAlt.png

Use the more complex mavgraph.py to graph the data

.. code-block:: bash

    mavgraph.py MyLogFile.BIN EKF1.PN NTUN.PosX*0.01

This example compares the EKF estimated North-South position from home vs the older Inertial Nav estimated position. See image at the top of this page for the resulting graph.

Changing parameters
===================

Simulation parameters may be changed before replaying a log using the option: ``-pNAME=VALUE`` (this sets the parameter ``NAME`` to
``VALUE``). 
The parameters which may be edited are those listed by running the `:ref:`param show`` command in `SITL <setting-up-sitl-on-linux>`.

For example, to change the EKF velocity delay parameter from 220ms to 400ms, run the command:

.. code-block:: bash

    ./build/linux/tools/Replay -- -pEKF_VEL_DELAY=400 MyLogFile.Bin
