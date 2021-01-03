.. _mission-planner-simulation:

==========================
Mission Planner Simulation
==========================

The Simulation tab provides a SITL (Software in the Loop) simulation capability. Many typical frame types for each Vehicle type have been created. This allows you to see the expected behavior for vehicles in missions, or with a joystick attached, actually be able to fly/drive the vehicle simulation as if with RC. This allows you to see potential effects of parameter changes on vehicle behavior or explore mission generation, without risking your vehicle first.


Demonstration Video

..  youtube:: XY2mnqYl9a0
    :width: 100%

Setup and Use
=============

The Simulation uses the same SITL models as those used in the Linux ``sim_vehicle.py`` script (see :ref:`dev:sitl-simulator-software-in-the-loop`, and uses the current master development branch of the code for firmware.

- Open Mission Planner's **Simulation** tab.
- Choose your vehicle type and frame (the "Model" drop down box). First select the "Model" frame type, then click the vehicle it corresponds to. If no "Model" (frame) is selected, then a normal plane will be used no matter what vehicle is clicked. If a frame is selected and a vehicle clicked that does NOT correspond to it, undefined results may occur.
- Default parameters for that vehicle/frame will be loaded and the simulation started. Mission Planner can be used to control the vehicle, setup missions, alter parameters, etc.
- In addition to the normal vehicle parameters, you can also now see and change the "SIM_x" (simulation) parameters via the CONFIG/Full Parameter Tree page. See :ref:`dev:sitl-parameters` for a list of these.

.. tip:: In addition, using the ``Extra command line`` box on the SIMULATION tab, you can start the SITL at any given location using the command ``--home={location}`` where location can be either gps coordinates,altitude ASL and vehicle initial heading, or a named location in the `Autotest locations file <https://github.com/ardupilot/ardupilot/blob/master/Tools/autotest/locations.txt>`__

::

    --home=CMAC
    --home=-35.363261,149.165230,584,353
