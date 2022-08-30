===============
Antenna Tracker
===============

.. code:: bash

    module load tracker
    
This is an improved antenna tracker, where the APM is mounted on an
antenna tracker (with a GPS module) which in turn is connected to the
ground station. Using the positions of the APM on the UAV and the APM on
the antenna, this module will send commands to the antenna APM to alter
its azimuth/elevation (via servos on the APM) to maintain tracking with
the UAV.

Use ``tracker set port <portname.``` and ``tracker set baudrate <n>``` to 
set the serial port and baudrate of the tracker.

Use ``tracker arm`` and ``tracker disarm`` to arm and disarm the servos
on the tracker respectively.

``tracker level`` will perform the level calibration routine and
``tracker calpress`` will calibrate the barometer, so that air pressure
corrections can be sent back to the UAV (for a more accurate altitude
estimation).

``tracker start`` will start the tracking.

There are several parameters for the tracker, to specify the serial port
that the antenna tracker APM is on. Use
``tracker param [set|show|fetch|help]`` to manage the parameters. There
are three parameters - port, baud and debug.

