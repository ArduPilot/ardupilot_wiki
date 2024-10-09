.. _flight-options:

==============
Flight Options
==============


:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>` is a bitmask that allows configuring several alterations to Copter's behavior.


=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
0                                       Disables the thrust loss check. See :ref:`thrust_loss_yaw_imbalance`
1                                       Disables the yaw imbalance warning. See :ref:`thrust_loss_yaw_imbalance`
2                                       Release the gripper on thrust loss.
3                                       Require position for arming (for every mode, not just for those requiring it)
=====================================   ======================

Default is no options enabled ("0"). Setting the bit will enable that function.