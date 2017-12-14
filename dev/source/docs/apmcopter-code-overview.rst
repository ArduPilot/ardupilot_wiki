.. _apmcopter-code-overview:

======================
Code Overview (Copter)
======================

The `code <https://github.com/ArduPilot/ardupilot>`__ is made up
of `the main Copter code <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter>`__ which
resides in it's own directory, and `the libraries <https://github.com/ArduPilot/ardupilot/tree/master/libraries>`__ which
are shared with Plane and Rover.

Below is a highlevel view of the ardupilot architecture.

.. image:: ../images/ArduPilot_HighLevelArchecture.png
    :target: ../_images/ArduPilot_HighLevelArchecture.png

Below is a more zoomed in view (as compared to the above diagram) of the architecture.

.. image:: ../images/copter-architecture.png
    :target: ../_images/copter-architecture.png

Click on the images below to see a high level view of flight-mode to
motor output code:

.. image:: ../images/AC_CodeOverview_ManualFlightMode.png
    :target: ../_images/AC_CodeOverview_ManualFlightMode.png

.. image:: ../images/AC_CodeOverview_AutoFlightModes.png
    :target: ../_images/AC_CodeOverview_AutoFlightModes.png
