.. _finding-the-particular-commit-which-introduced-a-bug:

====================================================
Finding the Particular Commit which Introduced a Bug
====================================================

This article explains how to perform a firmware binary `bisection search <https://en.wikipedia.org/wiki/Bisection_method>`__. This
is an efficient technique to find the particular build/commit that
introduced a reproducible bug.

.. tip::

   Knowing the build in which a defect was first introduced can help
   identify possible causes of the problem, and inform analysis of logs and
   other debug techniques. 

Firmware builds are available from
`firmware.ardupilot.org <https://firmware.ardupilot.org/>`__ for each
vehicle-type: \ `Copter <https://firmware.ardupilot.org/Copter/>`__,
`Plane <https://firmware.ardupilot.org/Plane/>`__,
`Rover <https://firmware.ardupilot.org/Rover>`__,
`AntennaTracker <https://firmware.ardupilot.org/AntennaTracker/>`__. To
perform a bisection search:

#. First test the firmware build half-way between the known working and
   failing builds. The result gives you a new "known working" or "known
   failing" build and halves the number of builds that must be tested.
#. Repeat this test process, each time halving the size of the test
   range, until the problem build has been identified.

Once you have identified the problem issue, note the build number and
include this in your bug report.

.. note::

   -  The autotest system generates a new firmware build for every commit
      to the source tree (this is a slight simplification, as commits may
      be batched if they arrive while a previous build is being tested).
      This means that if an error is reproducible it is possible to
      identify the specific commit(s) in which it first occurs.
   -  Given the number of ArduPilot firmware builds it should be possible
      to locate the problem build/commit within around 10 tests (at time of
      writing).
