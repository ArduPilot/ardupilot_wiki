=================
Gimbal Management
=================

.. code:: bash

    module load gimbal
    
The gimbal module manages any gimbals attached to the APM via the RC
output ports.

A gimbal menu is available on the GUI console.

gimbal mode
===========

Changes the mode of the gimbal to 1 of 3 options:

- GPS - point to a particular longitude/latitude
- MAVLINK - manual control via MAVLink commands
- RC - manual control via RC channels

.. code:: bash

    gimbal mode GPS

gimbal rate ROLL PITCH YAW
==========================

Specify the maximum rotational rates (degrees/sec) that the gimbal is
limited to. Different rates can be used for different axes.

.. code:: bash

    gimbal rate 20 40 13

gimbal point ROLL PITCH YAW
===========================

When in MAVLink mode, tell the gimbal to point in a certain direction.
Angles are in degrees.

.. code:: bash

    gimbal point 0 85 45

gimbal roi
==========

When in GPS mode, tell the gimbal to point to a certain location. The
location is picked by clicking on the map.

.. code:: bash

    gimbal roi
    
gimbal roivel
=============

Similar to ``gimbal roi``, but the user is able to specify the maximum 
velocity and acceleration of the gimbal.

.. code:: bash

    gimbal roivel [VEL_NORTH VEL_EAST VEL_DOWN] [ACC_NORTH ACC_EASY ACC_DOWN]
    

gimbal status
=============

Output the current status of the gimbal.

.. code:: bash

    gimbal status

