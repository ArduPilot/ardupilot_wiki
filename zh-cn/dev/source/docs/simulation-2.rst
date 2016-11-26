.. _simulation-2:

==========
Simulation
==========

This section contains topics about ArduPilot code-testing using
simulation. Simulation allows for safe testing of experimental code and
settings.  Crashing software planes is a lot cheaper than crashing real
ones!

Two "classes" of simulation are provided (not all solutions are
available for all vehicles)

-  :ref:`Hardware In the Loop <hitl-simulators>` (HITL)
   simulation replaces the plane and the environment with a simulator.
   The simulator has a high-fidelity aircraft dynamics model and
   environment model (wind, turbulence, etc.).  The physical APM
   hardware is configured exactly as for flight, and connects to your
   computer running the simulator, rather than the aircraft.
-  :ref:`Software In The Loop (SITL) <sitl-simulator-software-in-the-loop>`
   simulation (additionally) virtualizes the autopilot hardware as well
   as the aircraft and the environment as in HITL.  SITL is useful for
   rapid development and when physical hardware (autopilots and ground
   stations) are not available or not required.

   .. tip::

      We recommend :ref:`Software In The Loop (SITL) <sitl-simulator-software-in-the-loop>`
      as the setup is generally easier, it doesn't require vehicle
      hardware, and it supports all our main vehicle types.

In addition, you can also use:

-  **Log replay**: You can :ref:`replay binary dataflash log files <testing-with-replay>` using
   the "Replay" system to examine the behaviour of some aspects of the
   ardupilot internals

.. toctree::
    :maxdepth: 1

    SITL Simulator <sitl-simulator-software-in-the-loop>
    HITL Simulators <hitl-simulators>
    Testing with Replay <testing-with-replay>
    The ArduPilot Autotest Framework <the-ardupilot-autotest-framework>