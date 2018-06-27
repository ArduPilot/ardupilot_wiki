.. _balance_bot-hardware: 

==============
Hardware Setup
==============

.. tip:: Look up the rover hardware :ref:`assembly<rover-assembly-instructions>` page for more detailed information and instructions.

Required Parts
--------------
Take a look at the rover :ref:`introduction<gettit>` page for the full list of hardware. This list contains only the additional balance-bot specific/testing specific requirements. A full list of parts with instructions for assembly will be put up as the project progresses.

.. warning:: We are ourselves, only experimenting with the hardware right now. Do not consider this list as complete or final. 


#. Balance Bot Chassis(http://nevonexpress.com/Self-Balancing-Robot-Chassis-Body-Diy.php or any similar one)
#. Brushed DC motor with encoder(We use 12V, 200rpm)
#. Motor driver/ Brushed ESC
#. Telemetry radio (Will save you a lot of headache)

Motors and Wheels
-----------------
The balancing depends on very fine adjustments of the wheel. So any backlash between the motor and wheels would lead to poor balancing performance. This is bad:

.. youtube:: 4Lkcze44W3E
    :width: 100%


Weight Distribution
-------------------
For good balancing, the center of gravity of the vehicle must be reasonably high compared to the wheel axis. Think of trying to balance a stick with a lump of clay on you palm. Similar reasoning here. This is done to increase the inertia. If the center of gravity was near the wheel axis, the inertia would be low and balancing would become incredibly difficult! 

To achieve this, we placed the battery at the top. How high depends on the power of your motors. If it too high, then the motors would have trouble balancing the vehicle. In our case, the total height was less than 30cm.









