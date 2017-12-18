.. _smartrtl-mode:

==============
Smart RTL Mode
==============

When switched into Smart RTL (aka Smart Return-To-Launch), like regular RTL, the vehicle will attempt to return to the position where it was last armed.

The "Smart" part of this mode is that it will retrace the path to return to home instead of returning directly home.  This can be useful if there obstacles between the vehicle and the home position.

-  The :ref:`RTL_SPEED <RTL_SPEED>` parameter can be used to set the speed (in meters/second) at which the vehicle will return to home.  By default this parameter is zero meaning it will use the :ref:`WP_SPEED <WP_SPEED>` or :ref:`CRUISE_SPEED <CRUISE_SPEED>`.

..  youtube:: V0bolTN8QRM
    :width: 100%