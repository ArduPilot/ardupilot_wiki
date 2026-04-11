.. _coverage-analysis:

=================
Coverage Analysis
=================

ArduPilot provides an easy way to run automated checks to generate coverage analysis.

These are used to produce a `periodically-updated report <https://firmware.ardupilot.org/coverage/>`__ on our firmware server.

coveralls.io also uses the same mechanisms to generate its `own report <https://coveralls.io/github/ArduPilot/ardupilot>`__

-------------------------
Running Coverage Analysis
-------------------------

Run the full suite with:

::

   ./Tools/scripts/run_coverage.py --full

This generates data files describing the coverage, but also creates a HTML report for easy perusal.

::

   firefox reports/lcov-report/index.html

----------------------
Improving the coverage
----------------------

Coverage is generated from the :ref:`autotest suite <the-ardupilot-autotest-framework>`, :ref:`unit tests <ardupilot-unit-tests>` and the :ref:`example sketches <learning-ardupilot-the-example-sketches>`.

Choose the most relevant of those three mechanisms to add new tests to.  Generally:
 - AP_Maths code should be done as unit tests
 - flight code should be tested with the autotest suite
 - sensor/external-device code can be tested by creating a simulator
