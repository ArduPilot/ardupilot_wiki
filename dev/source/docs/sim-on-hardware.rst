.. _sim-on-hardware:

======================
Simulation on Hardware
======================

ArduPilot's Simulation on Hardware is similar to features sometimes known as "Hardware in the Loop".  Contrary to many of these systems, however, the simulation is not run externally (with the resulting environmental data passed to the vehicle), rather the vehicle's own autopilot runs both the flight control software and the simulation.  ArduPilot's native SITL simulation environment is re-used for this purpose.

Simulation on Hardware allows checking that:

- mission structure is correct
- control surfaces move appropriately throughout a simulated mission
- physical failsafes (e.g. parachutes) function appropriately without putting a vehicle at risk
- communications infrastructure works in the face of real traffic

.. note::

   Simulation on Hardware is being improved all the time.  It is suggested you run the master branch ("latest" firmware) to be able to take advantage of any new features that have been added.

The firmware is loaded on the autopilot and a GCS attached. The GPS initial location data is provided by the simulation and can be changed with `SIM_OPOS_LAT`, `SIM_OPOS_LNG`, `SIM_OPOS_ALT` and `SIM_OPOS_HDG` parameters. Setup, arming, and flying the simulation occurs as if sim_vehicle.py and the associated physics models for the vehicle and frame was used.

Limitations
===========

-  Parameter space is shared between the real aircraft and the simulated aircraft.  This leads to problems with sensor calibration as the sensor suite differs between the real and virtual aircraft.  Thus parameters must be wiped and rewritten to the vehicle when moving between real and simulation firmware.
- While it is possible to run on flash-constrained (1MB) boards, many other features will need to be compiled out to allow it to fit.
- Currently only firmware for Cube Orange QuadCopter autopilots is available pre-compiled on the firmware server.

Firmware
========

We provide a pre-compiled binary for Cube Orange at `our firmware server <https://firmware.ardupilot.org>`__. It is available only in `Copter <https://firmware.ardupilot.org/Copter/latest/CubeOrange-SimOnHardWare/>`__ version, currently.

Note that several features have been removed from this firmware, including mount support.

Compiling your own Firmware
============================

Simulation on Hardware is not compiled into the ArduPilot firmware by default.

If you have setup an :ref:`ArduPilot development environment <dev:building-the-code>` you can add it to any ChibiOS hwdef file by adding the line `env SIM_ENABLED 1`.  Use the CubeOrange-SimOnHardware hwdef file (and default parameter file!) as a reference.

A typical build script is shown below:

.. code:: bash

    set -e
    set -x

    BOARD=MatekH743
    THISDIR=$(dirname $0)

    ./waf configure \
      --board=$BOARD \
      --extra-hwdef="$THISDIR/extra-hwdef-sitl-on-hw.dat" \  a typical file is shown `here <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/on-hardware/extra-hwdef-sitl-on-hw.dat>`__
      --default-param="$THISDIR/default.param" \ a typical default param file is shown `here <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/on-hardware/default.param>`__

    ./waf copter --upload
    
    
Executing the Simulation
========================

Back up your existing parameters, missions, rally points, fences, signing keys and anything else you can think of.

Flash the new firmware as you would normally.

Reset the parameters by setting `FORMAT_VERSION` to zero and rebooting the vehicle.

Provide RC input as normal to the vehicle - or use mavlink RC overrides to provide RC input.

Wait for the vehicle to be ready to arm.  Try flying it as normal monitoring the GCS, including graphing actuator outputs, etc.


Allowing actuators to move
==========================

.. warning::

   Remove/disable all props or other dangerous actuators before continuing.  Be prepared to cut power to actuators should it become evident control surfaces are straining.  Keep clear of entrapment or other hazards.

Actuators can be permitted to... actuate.... by setting the `SIM_OH_MASK` parameter.  The bits correspond to the servo output channels, so to allow the first 4 channels to move set `SIM_OH_MASK` to 15.
