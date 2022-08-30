=============
Long Commands
=============

.. code:: bash

    module load cmdlong
    
This module is a collection of ``command_long_send`` functions that may 
be useful to the user.

Control
=======

.. code:: bash

    setspeed N
    
If in automated flight mode, set the vehicles target speed to ``N`` m/s.


.. code:: bash

    setyaw ANGLE ANGULAR_SPEED MODE
    
Set the target yaw of the vehicle to ``ANGLE`` (0-360), with a maximum angular speed 
of ``ANGULAR_SPEED`` (deg/sec). ``MODE`` is either 0 (absolute angle) or 1 (relative angle).

.. code:: bash

    takeoff ALTITUDE_IN_METERS

Sends an automated takeoff command to the vehicle. It will consider the takeoff finished at 
``ALTITUDE_IN_METERS`` m (relative).

.. code:: bash

    velocity X Y Z
    
Sets a desired vehicle velocity in a local north-east-down (x, y, z) coordinate frame. 
All velocities in m/s.

.. code:: bash

    position X Y Z
    
Sets a desired vehicle position in a local north-east-down (x, y, z) coordinate frame. 
All positions in m.

.. code:: bash

    attitude Q0 Q1 Q2 Q3
    
Set the desired attitude of the vehicle in quaternion format.

.. code:: bash

    posvel N E D
    
Sets the target position according to the previous mouse click on the map and the target 
velocity in North-East-Down format (m/s). Uses a global frame.

Camera
======

Note the following commands are not processed by APM, rather they are meant for a companion computer which controls the camera.

.. code:: bash

    cammsg M1 M2 M3 M4 M5 M6 M7 M8
    
Send a MAV_CMD_DO_DIGICAM_CONTROL Mavlink message. Parameters are: M1=Session control e.g. show/hide lens, 
M2=Zoom's absolute position, M3=Zooming step value to offset zoom from the current position, 
M4=Focus Locking, Unlocking or Re-locking, M5=Shooting Command, M6=Command Identity, M7=Empty.

.. code:: bash

    cammsg_old
    
Send an old-style MAV_CMD_DO_DIGICAM_CONTROL shooting command.

.. code:: bash

    camctrlmsg M1 M2 M3 M4 M5 M6 M7
    
Send a MAV_CMD_DO_DIGICAM_CONFIGURE Mavlink message. The Parameters are: M1=Modes [P, TV, AV, M], M2=Shutter Speed, M3=F Stop number, M4=ISO Number, M5=Exposure type, M6=Command Identity, M7=Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off).

Systems
=======

.. code:: bash

    parachute ACTION
    
Sends an action to the parachute. ``ACTION`` can be enable, disable or release.

.. code:: bash

    engine start
    engine stop
    engine M1, M2, M3
    
Sends an engine control command (MAV_CMD_DO_ENGINE_CONTROL). It can start or stop the engine. 
Otherwise the full set of options can be specified, with M1=1 or 0 for engine start/stop, M2=	0: Warm start, 1:Cold start. Controls use of choke where applicable, M3=Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height.

.. code:: bash

    cmdlong COMMAND OPTIONS
    
Send a general MAV_CMD_LONG message to the vehicle. ``COMMAND`` is the name of the command. The options
follow in ``[arg1] [arg2] ...`` format.


