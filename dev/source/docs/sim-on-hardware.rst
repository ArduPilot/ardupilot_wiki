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

The firmware is loaded on the autopilot and a GCS attached. The GPS initial location data is provided by the simulation and can be changed with :ref:`SIM_OPOS_LAT<SIM_OPOS_LAT>`, :ref:`SIM_OPOS_LNG<SIM_OPOS_LNG>`, :ref:`SIM_OPOS_ALT<SIM_OPOS_ALT>` and :ref:`SIM_OPOS_HDG<SIM_OPOS_HDG>` parameters. Setup, arming, and flying the simulation occurs as if sim_vehicle.py and the associated physics models for the vehicle and frame was used.

Limitations
===========

-  Parameter space is shared between the real aircraft and the simulated aircraft.  This leads to problems with sensor calibration as the sensor suite differs between the real and virtual aircraft.  Thus parameters must be wiped and rewritten to the vehicle when moving between real and simulation firmware.
- Currently pre-compiled firmware for only Cube Orange QuadCopter autopilots is available on the firmware server. The instructions below will allow firmware to be compiled for other boards and vehicle types

Save your parameters!
=====================

If you have been using the autopilot, be sure to save the parameters, mission items, fences, and rally point to files to be able to reload them later.

Once you have saved these to replace them after simulating, reset the changed parameters by setting :ref:`FORMAT_VERSION<FORMAT_VERSION>` parameter to zero and rebooting the vehicle. This allows new default to be used in the simulation.

.. note:: with MAVProxy, once rebooted you need to "FETCH" params to refresh the parameters to see them changed back to defaults, or dis-connect and re-connect to the autopilot.

Pre-Compiled Firmware
=====================

A pre-compiled binary for Cube Orange is provided at `our firmware server <https://firmware.ardupilot.org>`__. It is available only in `Copter <https://firmware.ardupilot.org/Copter/latest/CubeOrange-SimOnHardWare/>`__ version, currently.

Note that several features have been removed from this firmware, including mount support.

Compiling your own Firmware and Loading to AutoPilot
====================================================

Simulation on Hardware is not compiled into the ArduPilot firmware by default.

If you have setup an :ref:`ArduPilot development environment <dev:building-the-code>` you can add it to any ChibiOS hwdef file by adding the line ``env SIM_ENABLED 1``.  Use the CubeOrange-SimOnHardware hwdef file (and default parameter file!) as a reference.

An flexible firmware building script is located `here <https://github.com/ArduPilot/ardupilot/tree/master/Tools/scripts/sitl-on-hardware/sitl-on-hardware.py>`__, which uses existing board's hwdef and builds the firmware with the simulator.I

- first configure the build (example shown for CubeOrange, for QuadPlane Tilt Tricopter) and upload to board

.. code:: bash

    ./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board CubeOrange-SimOnHardWare --vehicle plane --simclass QuadPlane --frame quadplane-tilttri --defaults ./Tools/autotest/default_params/quadplane-tilttri.parm --upload

- board: board target for firmware
- vehicle: plane/rover/copter/blimp, same as used by sim_vehicle.py
- simclass: Plane/Multicopter/Helicopter/SimRover/Sailboat/QuadPlane
- frame: same as used by sim_vehicle.py, this modifies the physics model according to which frame type is used, but unlike software based SITL it does not also load a defualt set of params for this frame (like :ref:`Q_TILT_ENABLE<Q_TILT_ENABLE>` = 1, etc.). So they must be manually added after the firmware is loaded.
- defaults: extra defaults file (optional). As shown,it will load the default param file that sim_vehicle.py loads for the example frame. If omitted, a minimal defaults.parm file is loaded to get most vehicles simulating. You may want to create a file that sets up these necessary parameters for the simulation (see below), the frame defaults, or the ones you are using on your vehicle including the other parameters for specific hardware devices you may wish to activate during the simulations(be sure not to change the required parameters below).

In order for the simulation to work and emulate flight while the autopilot remains stationary, it will use simulated IMUs, GPS, and Compass.

Parameters needed for Simulation
================================

Connect a GCS.

In order for the simulation to work, the IMUs, GPS, and Compass sensors must be simulated. The script above sets up these sensors so you do not have to, as well as disabling the safety switch and IMU heater limits :

- :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 10
- :ref:`GPS1_TYPE<GPS1_TYPE>` = 100
- :ref:`SIM_MAG1_DEVID<SIM_MAG1_DEVID>` =  97539

You may also need to have the correct parameters for your specific vehicle frame, as mentioned above.

Starting at you home field
==========================

You can set the simulation starting location by setting the following parameters, otherwise the normal Canberra location will be used:

- :ref:`SIM_OPOS_LAT<SIM_OPOS_LAT>`  in the form of -/+ X.xxx
- :ref:`SIM_OPOS_LNG<SIM_OPOS_LNG>`
- :ref:`SIM_OPOS_ALT<SIM_OPOS_ALT>`  in meters AMSL
- :ref:`SIM_OPOS_HDG<SIM_OPOS_HDG>`  0-360 deg for initial heading

Reboot and reconnect the GCS. The simulation will be running.

Using the Simulator
===================

The simulator will behave almost identically to the SITL simulation on a PC. The autopilot, is now running the sims physics models and simulated sensors. You will have to load parameters for your outputs, and any other none default parameters for your specific vehicle, like relays, desired failsafe actions, etc. you can specify a defaults file in the above script, or load them manually after the firmware is loaded.

In order to arm the vehicle, provide RC input as normal to the autopilot - or use MAVLink RC overrides to provide RC input. Just setting "rc 3 1000" in MAVProxy will clear the RC not present pre-arm checks, and you can use the ``rc x <pwm value>`` commands in place of RC, if you wish, or use an RC receiver.

.. note:: Copters require that the rudder input be sent or an RC override setting it to neutral (ie rc 4 1500) in order for the yaw controller to operate properly

Wait for the vehicle to be ready to arm.  Try flying in the sim just as if it were being software simulated,  monitoring the GCS, including graphing actuator outputs, etc.

A video showing build and operation is here:

.. youtube:: 81FKqNB6C38


Allowing actuators to move
==========================

.. warning::

   Remove/disable all props or other dangerous actuators before continuing.  Be prepared to cut power to servos should it become evident control surfaces are straining.  Keep clear of entrapment or other hazards.

Servo/Motor outputs can be permitted to operate by setting the :ref:`SIM_OH_MASK<SIM_OH_MASK>` parameter.  The bits correspond to the servo output channels, so to allow the first 4 channels to move set :ref:`SIM_OH_MASK<SIM_OH_MASK>` to 15.

Relay outputs can be permitted to operate by setting the :ref:`SIM_OH_RELAY_MSK<SIM_OH_RELAY_MSK>` parameter. The bits correspond to the relays, so to allow the first 2 relays to actuate set :ref:`SIM_OH_RELAY_MSK<SIM_OH_RELAY_MSK>` to 3.
