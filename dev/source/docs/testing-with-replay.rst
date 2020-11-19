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

Checking that new code has no effect on the EKF
===============================================

When modifying the EKF code it can be useful to confirm your changes have no impact on the EKF's estimates in some situations.  This can be done by following the procedure below:

- Start SITL with a vehicle of your choice, using "master"
- Set these parameters and optionally some sensor position parameters (i.e GPS_POS_X,Y,Z) to non-zero values

.. code-block:: bash

    param set LOG_DISARMED 1
    param set LOG_REPLAY 1
    param set GPS_POS1_Y 0.1
    param set SIM_GPS_POS1_Y 0.1

- Fly the vehicle for a short flight which includes fast forward flight and turns
- Land the vehicle and download the onboard log (i.e. 00000001.BIN)
- Build Replay (see "Building Replay" above)

.. code-block:: bash

    cd ardupilot
    ./waf replay

- Process the onboard log with Replay (see "Using Replay" above):

.. code-block:: bash

    build/sitl/tools/Replay 00000001.BIN

- Move the resulting log file to a safe place

.. code-block:: bash

    mv logs/00000001.BIN replay-00000001.BIN

- Checkout the new branch

.. code-block:: bash

    git checkout <new-branch>

- Build Replay again (see "Building Replay" above)

- Use the check_replay.py script to check that there are no changes:

.. code-block:: bash

    ../Tools/Replay/check_replay.py replay-00000001.BIN

- if nothing has changed a message like below will be displayed

.. code-block:: bash

    Processing log replay-00000001.BIN
    Processed 30166/30166 messages, 0 errors
    Passed
