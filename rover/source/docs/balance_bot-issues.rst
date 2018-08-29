.. _balance_bot-issues: 

========================
Commmon Issues and Fixes
========================

This page details some of the most common issues with Balance Bots and their fixes.

The Bot Balances, but wobbles too much
--------------------------------------
If your Balance Bot is able to maintain balance but wobbles a lot, or moves back and forth randomly trying to balance then your Bot must be having one of the following issues. Take a look at this video, if you are not sure if this is your problem:

 .. youtube:: -EESMnSEpeM
    :width: 100%

1) Motor Backlash
+++++++++++++++++

The balancing depends on very fine adjustments of the wheel. So any backlash between the motor and wheels would lead to poor balancing performance. 

This is bad:

.. youtube:: 4Lkcze44W3E
    :width: 100%

The only fix is to change your motors. Make sure the backlash is as less as possible.

2) Weight Distribution
++++++++++++++++++++++
For good balancing, the center of gravity of the vehicle must be reasonably higher than the axis of the wheels. 

A good way to get the weight distribution right is to keep the batteries at the top of the Balance Bot, instead of near the wheels like in other rover frames.

How high depends on the power of your motors. If it too high, then the motors would have trouble balancing the vehicle. In our Balance Bot, the total height was less than 30cm.

3) Minimum Throttle
+++++++++++++++++++
If the minimum throtte sent to the motors from the Flight Controller is **too less** to turn on the motors, then your Balance Bot will move back and forth struggling to keep balance. Sometimes this issue can also cause it to go off in one direction and topple.

If the minimum throttle value is **too high** then your Balance Bot will be able maintain balance and stand on spot, but will be very shaky. 

Refer to this guide to set the minimum throttle value correctly.


4) PID tuning
+++++++++++++
No amount of PID tuning can fix wobbling if the above two reasons are not handled. So please verify those before proceeding. Refer to the tuning page for details on PID tuning.


The Bot always moves off in one direction
-----------------------------------------
This issue can be because the **minimum throttle** is not set correctly. Refer to the instructions above to set it.

This can also be because the **accelerometer calibration** is invalid. Disturbing the flight controller, changing connections, crashes etc can affect the acceleromter calibration. Often, a one axis trim should fix this. To perform a one axis trim:

#. Keep your balance bot upright and level
#. In Mission Plaaner, go to Initial Setup -> Mandatory Hardware -> Accelerometer Calibration and click calibrate level
#. If you use MAVProxy instead,execute the command ahrstrim
#. Hold the Bot steady as the Flight Controller light flashes red and blue

Other issues
------------
Many issues can arise because of incorrect parameter settings. Do verify that the parameters on your Bot are set as specified in the :ref:`configuration page<balance_bot-configure>`. 

If you encounter an issue, where the motor directions seem inverted or the Bot spins in circles, then recheck your connections. 

