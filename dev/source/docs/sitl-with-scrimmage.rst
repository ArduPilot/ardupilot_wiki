.. _sitl-with-scrimmage:

===================================
Using SCRIMMAGE as a SITL simulator
===================================
`Simulating Collaborative Robots in Massive Mulit-Agent Game Execution (SCRIMMAGE) <http://www.scrimmagesim.org/>`__
provides a flexible simulation environment for the experimentation and testing of novel mobile robotics algorithms.
SCRIMMAGE provides a three-dimensional robotics environment that can simulate varying levels of sensor and motion model
fidelity due to its flexible plugin interface. This allows a robotics researcher to simulate hundreds of aircraft with
low-fidelity motion models or tens of aircraft with high-fidelity motion models on a standard consumer laptop.

Installing SCRIMMAGE
====================

To install SRIMMAGE for use in ArduPilot follow the build instructions in the
`SCRIMMAGE README on Github. <https://github.com/gtri/scrimmage/blob/master/README.md>`__


Starting an ArduPlane Simulation
================================

To start an ArduPlane simulation use the ``-f`` argument in sim_vehicle.py:
::

    cd ArduPlane
    ../Tools/autotest/sim_vehicle.py -f scrimmage-plane

Starting an ArduCopter Simulation
=================================

To start an ArduCopter simulation use the ``-f`` argument in sim_vehicle.py:
::

    cd ArduCopter
    ../Tools/autotest/sim_vehicle.py -f scrimmage-copter

Additional SCRIMMAGE Parameters
===============================

Additional parameters can be passed into SCRIMMAGE using the ``-A`` and ``--config`` arguments. These parameters can be used to
overwrite the defaults in the SCRIMMAGE mission file such as the motion model, visual model, or terrain.
::

    cd ArduPlane
    ../Tools/autotest/sim_vehicle.py -f scrimmage-plane -A "--config visual_model=zephyr-red,motion_model=FixedWing6DOF"

Valid Arduplane motion models include FixedWing6DOF and JSBSimControl. Valid visual models are zephyr-blue,
zephyr-red, and zephyr-green. Currently the only valid copter motion model is Multirotor and the iris visual model.
Additional visual and terrain models can be installed using the scrimmage apt repository added during installation.

::

    sudo apt install scrimmage-extra-visual-models
    sudo apt install scrimmage-extra-terrain
