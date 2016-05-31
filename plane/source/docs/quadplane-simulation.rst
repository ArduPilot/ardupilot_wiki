.. _quadplane-simulation:

QuadPlane Simulation
====================

A simple QuadPlane model is available in SITL, allowing you to test the
features of the QuadPlane code without risking a real aircraft.

You can start it like this:

::

    sim_vehicle.sh -j4 -f quadplane --console --map

To visualise the aircraft you can use FlightGear in view-only mode. The
simulation will output FlightGear compatible state on UDP port 5503.
Start FlightGear using the **fg_plane_view.sh** scripts provided in
the **Tools/autotest** directory.

Note that to get good scenery for FlightGear it is best to use a major
airport. I tend to test at San Francisco airport, like this:

::

    sim_vehicle.sh -L KSFO -f quadplane --console --map

Using the joystick module with a USB adapter for your transmitter gives
a convenient way to get used to the QuadPlane controls before flying.

If flying at KSFO there is a sample mission available with VTOL takeoff
and landing:

::

    wp load ../Tools/autotest/ArduPlane-Missions/KSFO-VTOL.txt

As usual you can edit the mission using "module load misseditor"
