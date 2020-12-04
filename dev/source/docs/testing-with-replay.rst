.. _testing-with-replay:

===================
Testing with Replay
===================

Introduction
============

Replay is a program that takes a dataflash log file and replays it
through any branch's code allowing for state estimation issues to
be analyzed using that branch's code instead of the code used while generating the dataflash log.

.. note:: The log structure of the firmware used to create the log and the branch to be tested with it must be the same.

.. note:: In order to use Replay the log must be generated with `LOG_REPLAY` set to 1. And it is preferred to also have `LOG_DISARMED` also set to 1, to obtain the most information in the log.

.. image:: ../images/Replay_EKFVsINAV.png
    :target: ../_images/Replay_EKFVsINAV.png

Building Replay
===============

On your Linux or Ubuntu machine, from the root directory of an ArduPilot repository, using the branch you wish to replay the log through:

.. code-block:: bash

    ./waf configure --board=sitl --debug   //--debug is optional but allows using a debugger, if desired when analyzing issues
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

This will produce an output file xxx.BIN in the folder "./logs", which will be the highest numbered log since it was just created.

Use MAVExplorer to graph the data. Both the original and Replay generated EKF messages will be included in the log data. For example, instead of possible graphs for IMU0 and IMU1's EKF2- NKF2 message items:  "NKF2, NKF2[0], and NKF2[1]", there will also be "NKF2[100] adn NKF2][101]" graph groups for the replay generated log messages.

.. note:: if using WSL, do not reference the log file via the external Windows WSL path since this will be extremely slow. Instead copy the log from the Windows file system directly into the WSL environment and execute Replay on it within that environment.

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
- Move the log to a safe place

.. code-block:: bash

    mv logs/00000001.BIN test-00000001.BIN

- Checkout the new branch and build Replay (see "Building Replay" above)

.. code-block:: bash

    git checkout <new-branch>
    cd ardupilot
    ./waf replay

- Process the onboard log with Replay (see "Using Replay" above)

.. code-block:: bash

    build/sitl/tools/Replay test-00000001.BIN

- Move the resulting log file to a safe place.  This new log contains all the information of the original log plus what the new-branch's EKF would have produced

.. code-block:: bash

    mv logs/00000001.BIN replay-00000001.BIN

- Use the check_replay.py script to check that there are no changes:

.. code-block:: bash

    ../Tools/Replay/check_replay.py replay-00000001.BIN

- if nothing has changed a message like below will be displayed

.. code-block:: bash

    Processing log replay-00000001.BIN
    Processed 30166/30166 messages, 0 errors
    Passed


Ensuring EKF changes have had no effect on its output
=====================================================

Often changes to the EKF are expected to have no functional change.  Refactoring, removing dead code, adding comments, rearranging parameters, changing function names and the like.

If you are making such a change, ``Tools/Replay/check_replay_branch.py`` is provided to ensure your current branch does not change the EKF's output, as tested by the autotest suite's Replay tests.

It:
  - generates a Replayable log on the master branch
  - compiles and runs Replay on your branch
  - uses ``Tools/Replay/check_replay.py`` to ensure the EKF output has not changed

e.g.

.. code-block:: bash

    pbarker@bluebottle:~/rc/ardupilot(pr/move-gsf-logging-ekf2)$ ./Tools/Replay/check_replay_branch.py
    chdir (/home/pbarker/rc/ardupilot)
    lckfile='/home/pbarker/rc/buildlogs/autotest.lck'
    step=build.Copter
    step=test.Copter.Replay
    Running: ("git rev-parse HEAD") in (/home/pbarker/rc/ardupilot)
    >>>> RUNNING STEP: build.Copter at Tue Dec  1 13:26:32 2020
    Running: ("/bin/rm -f logs/*.BIN logs/LASTLOG.TXT") in (.)
    'build' finished successfully (4m26.874s)
    .
    .
    .
    >>>> PASSED STEP: build.Copter at Tue Dec  1 13:31:03 2020
    >>>> RUNNING STEP: test.Copter.Replay at Tue Dec  1 13:31:03 2020
    Running: ("/bin/rm -f logs/*.BIN logs/LASTLOG.TXT") in (.)
    step=test.Copter.Replay
    .
    .
    .
    AT-0298.3: Stopping SITL
    >>>> PASSED STEP: test.Copter.Replay at Tue Dec  1 13:36:01 2020
    Processing log logs/00000004.BIN
    Processed 66495/66495 messages, 0 errors
    pbarker@bluebottle:~/rc/ardupilot(pr/move-gsf-logging-ekf2)$ 
