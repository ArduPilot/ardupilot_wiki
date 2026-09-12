. _sitl-with-skysim:

======================
Using SITL with SkySim
======================

`SkySim <https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git>`__ is an open-source multirotor flight simulator that
can act as a physics backend for SITL through the :ref:`JSON interface
<sitl-with-JSON>`. SkySim provides the flight dynamics (blade-element rotor
model, ground effect, vortex-ring state, ISA atmosphere, Dryden turbulence and
a battery model) and the simulated sensors, while ArduPilot runs as the flight
controller. It runs natively and also entirely in the browser via WebAssembly,
so an autonomy stack can be exercised against SITL without a local install.

Supported Vehicles
===================

The steps on this page use a quadcopter in X configuration. SkySim maps the
first four servo channels to its rotors and returns IMU, position, velocity and
attitude to ArduPilot every physics step.

Installation
============

SkySim is distributed from its `repository <https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git>`__. Follow the
quick-start there to run either the native build or the browser build. No
ArduPilot-specific build options are required; the JSON backend is selected at
run time.

Setting Up the Simulator
========================

#. Start SkySim and select the **ArduPilot (JSON)** bridge.
#. Set the physics rate to at least the vehicle loop rate (400 Hz for copter is
   recommended).
#. SkySim then listens for ArduPilot on UDP port 9002 and auto-detects the
   controller from the first frame it receives — no target address needs to be
   configured on the SkySim side.

Starting SITL
=============

Start SITL on the machine where ArduPilot is built, pointing the JSON backend
at the machine running SkySim:

.. code-block:: bash

    sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 --console --map

Use ``127.0.0.1`` when SITL and SkySim run on the same machine, otherwise use
the address of the machine running SkySim.

Frame parameters can be set on the command line. For example, a quadcopter in X
configuration:

.. code-block:: bash

    sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 -P FRAME_TYPE=1

Once SITL has started, the vehicle can be armed and flown from MAVProxy, or from
any other ground station connected to SITL.

Running SITL in WSL
-------------------

WSL2 has its own virtual network, so ``127.0.0.1`` inside WSL does not reach a
simulator running on Windows. Get the address of the Windows host from WSL:

.. code-block:: bash

    ip route show default

which prints a line such as ``default via 172.20.0.1 dev eth0``, and pass that
address to SITL:

.. code-block:: bash

    sim_vehicle.py -v ArduCopter -f quad --model JSON:172.20.0.1

The Windows firewall must allow inbound UDP traffic on the simulator port.

Troubleshooting
===============

If ArduPilot keeps waiting for the simulator, check that SkySim has been started
with the **ArduPilot (JSON)** bridge selected, that the address given to
``--model JSON:`` is the machine running SkySim, and that inbound UDP traffic on
port 9002 is not blocked by a firewall. On first connection ArduPilot prints the
list of JSON fields it received; use it to confirm SkySim is sending every
required field.

Getting Help
============

Setup notes and the bridge reference are kept in the `SkySim documentation
<https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git>`__. Problems with the simulator can be reported on its `issue
tracker <https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git/issues>`__.
