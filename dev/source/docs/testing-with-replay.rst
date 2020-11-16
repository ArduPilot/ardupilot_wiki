.. _testing-with-replay:

===================
Testing with Replay
===================

Introduction
============

Replay is a program that takes a dataflash log file and replays it
through the latest master code allowing for state estimation issues to
be analysed with new code.

It is recommended that if a problem is reproducible that the log be
generated with both the `LOG_REPLAY` and `LOG_DISARMED` parameters set
to 1.

.. image:: ../images/Replay_EKFVsINAV.png
    :target: ../_images/Replay_EKFVsINAV.png

Building Replay
===============

On your Linux or Ubuntu machine, from the root directory of an ArduPilot repository:

.. code-block:: bash

    ./waf configure --board=sitl --debug
    ./waf replay

This will create a file called ``build/sitl/tools/Replay``.

Using Replay
============

Display the Replay help instructions:

.. code-block:: bash

    build/sitl/tools/Replay -h

Run a log through Replay to generate the plot and EKF data files:

.. code-block:: bash

    build/sitl/tools/Replay MyLogFile.BIN

This will produce an output file 1.BIN in the folder "./logs"

Use MAVExplorer to graph the data

Changing parameters
===================

Simulation parameters may be changed before replaying a log using the option: ``-parm NAME=VALUE`` (this sets the parameter ``NAME`` to
``VALUE``). 
The parameters which may be edited are those listed by running the `:ref:`param show`` command in `SITL <setting-up-sitl-on-sitl>`.

For example, to change the EKF I gate value to 1,000, run the command:

.. code-block:: bash

    build/sitl/tools/Replay --parm EK2_VEL_I_GATE=1000 log_1.bin
