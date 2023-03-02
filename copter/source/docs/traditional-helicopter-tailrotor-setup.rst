.. _traditional-helicopter-tailrotor-setup:

=========================================
Traditional Helicopter – Tailrotor Setup
=========================================

General section on tailrotors outlining the different options for tailrotor setup



Tailrotor setups
================

Servo Only
++++++++++


Direct Drive Fixed Pitch
++++++++++++++++++++++++


Direct Drive Variable Pitch
+++++++++++++++++++++++++++



Collective to Tailrotor Compensation
====================================

Collective to tailrotor compensation is used to remove the effects of the aircraft yawing when collective pitch is changed.  

In versions 4.3 and earlier, the parameter H_COLYAW was used.  This implementation assumed the tailrotor changed linearly with collective blade pitch.  In versions 4.4 and later, the parameter H_COL2YAW is used.  This implementation uses an accepted helicopter performance relationship between helicopter power required and weight.  Setting this parameter will only be valid for one rotor speed.  Set the tailrotor pitch so it is zero deg blade pitch at the SERVOX_TRIM value. if the rotor speed is changed then the parameter might require retuning.  The relationship uses collective to the 3/2 power to determine the tailrotor correction.  It is recommended to start at 0.5 and increase the parameter until there is little to no yawing when changing the collective pitch.  One other way would be to determine the yaw required for hovering as well as the collective and then calculate the value.

.. toctree::
    :hidden:

    traditional-helicopter-aerobatic-setup
