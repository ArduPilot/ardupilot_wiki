.. _sitl-with-webots-python:

=============================
Using SITL with Webots Python
=============================

ArduPilot's python-based Webots implementation is a cross-platform simulation 
tool that allows running ArduPilot without risking real-life crashes. This page
goes over how to set up and use Webots with ArduPilot's SITL as well as 
touching on how to create new vehicles and worlds.

.. youtube:: 6K4RULy1cD4
    :width: 100%

|

------------
Installation 
------------
Webots itself can be downloaded from 
`www.cyberbotics.com <https://www.cyberbotics.com/>`__. If there is any trouble
with installation see 
`Webots' install guide <https://cyberbotics.com/doc/guide/installation-procedure>`__
for further guidance.

.. note:: 
   The Webots python implementation was built for Webots 2023a and is not 
   backward compatible. Newer versions should also work, however.

------------------------
Running the Iris Example
------------------------
The following assumes you have already successfully set up your ArduPilot dev 
environment (see 
`Building the code <https://ardupilot.org/dev/docs/building-the-code.html>`_).

1. Run Webots and open 
   ``/path/to/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt``
   (``File`` > ``Open World``). If everything is working you should see the 
   simulation time advancing and no errors in the console 
   (red warnings are fine).
2. Run the SITL with the following command, replacing ``/path/to/`` 
   with your local absolute path to ardupilot::

      /path/to/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=/path/to/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm

.. warning:: 
   When running SITL in WSL2 and Webots in Windows you will have to 
   provide the SITL with Windows' IP by adding something like 
   ``--sim-address=172.x.x.1`` to the command above. Your exact IP can be found
   by running ``ipconfig`` in cmd and looking for the IP under the WSL adapter.
   You may also need to add WSL's IP found with ``hostname -I`` to ``iris.wbt``
   so it looks something like this::

      Iris {
         controller "ardupilot_vehicle_controller"
         controllerArgs [
            "--motors"
            "m1_motor, m2_motor, m3_motor, m4_motor"
            "--sitl-address"
            "172.x.x.x"
         ]
      }


If all goes well you should see ``Connected to ardupilot SITL`` in the Webots 
console. At this point, you should be able to use MAVProxy (in the SITL terminal 
window) or a GCS to fly the drone around. 

--------------------
Other Example Worlds
--------------------

- ``iris.wbt`` implements a basic quadcopter. 
- ``iris_camera.wbt`` implements a camera on a quadcopter. See  
  ``example_camera_receive.py`` in the scripts folder for one way to get 
  access to the camera.
- ``iris_depth_camera.wbt`` implements a camera that sees depth rather than color. 
  The example camera script can also receive these images.
- ``pioneer3at.wbt`` implements a pioneer3at rover. 
  For the SITL command, run it with ``Rover`` instead of ``ArduCopter`` and
  ``pioneer3at.parm`` instead of ``iris.parm``.
- ``crazyflie.wbt`` implements a crazyflie quadcopter.
  Run the SITL command with ``crazyflie.parm`` instead of ``iris.parm``.
- ``crazyflie_double.wbt`` implements two crazyflie drones in a swarm.
  For the SITL command, run it with ``crazyflie.parm`` instead of 
  ``iris.parm`` and additionally add ``-n 2 --auto-sysid``.

--------------------
Creating a New World
--------------------
Creating a new world can be done by copying an example world (such as 
``iris.wbt``) and making desired changes. Prebuilt models (called 
`PROTOs <https://cyberbotics.com/doc/reference/proto>`__) can be added
directly through Webots by pressing ``CTRL+SHIFT+A``, or by adding them 
directly into the world file 
(`see options in documentation <https://cyberbotics.com/doc/guide/objects>`__).

If you want to go a bit further, check out 
`Webots' tutorials <https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots>`__.

.. note:: 
   An important parameter to set in every new world is the ``basicTimeStep`` 
   parameter in the WorldInfo object. This parameter should be set to 1 or 2,
   as it represents how many milliseconds the simulation should delay between
   physics timesteps. Setting this value any higher causes ArduPilot's main 
   loop to run too slow. 

----------------------
Creating a New Vehicle
----------------------
An ArduPilot vehicle in Webots is a 
`Robot <https://cyberbotics.com/doc/reference/robot>`__ object that uses the 
``ardupilot_vehicle_controller.py`` controller (found in the ``controllers`` 
directory). This controller is run when the simulation is started and is responsible 
for connecting to the SITL and passing it Webots sensor information. Note that 
to do this it assumes that the Robot object has an
`Accelerometer <https://cyberbotics.com/doc/reference/accelerometer>`__, 
`Gyro <https://cyberbotics.com/doc/reference/gyro>`__, 
`InertialUnit <https://cyberbotics.com/doc/reference/inertialunit>`__, 
and `GPS <https://cyberbotics.com/doc/reference/gps>`__.

To create a new vehicle that can be used in multiple worlds, like the Iris model,
we can create a `PROTO <https://cyberbotics.com/doc/reference/proto>`__ file
which will describe the vehicle (a Robot object) and be importable into any world.
The simplest way to do this is to copy the ``iris.proto`` file in the ``protos``
folder and edit or remove components (such as propellers, mesh, and the 
extension slot). If you want to start from scratch, you can look at the 
`documentation for creating a PROTO file <https://cyberbotics.com/doc/reference/proto-definition>`__.

Of particular note, the Robot object has a property called ``controllerArgs``
which are passed to the controller when the simulation is 
run, allowing us to specify traits of the vehicle such as motor names, count,  
and directions. The full list of available arguments are documented in 
``ardupilot_vehicle_controller.py``. 