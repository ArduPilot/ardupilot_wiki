.. _sailboat-hardware: 

Sailing functionality must first be enabled by setting :ref:`SAIL_ENABLE <SAIL_ENABLE>` to 1 and re-booting.

==============
Hardware Setup
==============

Sailing vehicles require a rudder for boats or movable wheel(s) for land vehicles. This is setup in the normal way by assigning a servo output channel, ``SERVOx_FUNCTION``, to function 26 (Ground Steering).

Although not absolutely required for best results a wind vane should be fitted to allow the vehicle to sense the wind 
direction. See :ref:`wind-vane`. If a wind vane sensor is not used, then assumed wind direction must be input using these methods: :ref:`wind-vane-home-heading`.

A further improvement can be gained by fitting a wind speed sensor, the faster your vehicle goes relative 
to the wind speed the more important this is. See :ref:`wind-vane`. 

.. image:: ../images/sailboat-windvane.png
    :target: ../_images/sailboat-windvane.png

Sailing vehicles also require a method of controlling the sails. There are three supported actuator types, all actuators on a single vehicle should be of the same type.


Sail winch
----------
The servo must pull in and let out the sails in a symmetrical way. I.e. no change of servo position should be required to set the sails at the same angle on a new tack. This is typically 
achieved through the use of a sail winch servo with a main sheet system. The sail winch is servo output function, ``SERVOx_FUNCTION``, 89 - Main Sail.

When using a sail winch the sail limits angle must be configured. 
This can be tested by arming in manual mode; the throttle will directly control the sail position.
Throttle stick down (towards you) should result in the sail being sheeted in towards the centre line.
If the sail is sheeted out the servo should be reversed. The servo min and max parameters can then be
used to set up the range of travel. The min and max values should be set such that the boom is brought
in towards the centre line of the boat but not pulled down tightly. The boom should be able to be let
out until it reaches the shrouds; if no shrouds are fitted the boom shouldn’t go too far past 90 degrees
to the boat centre line. 

The sail angle :ref:`SAIL_ANGLE_MIN<SAIL_ANGLE_MIN>` and :ref:`SAIL_ANGLE_MAX<SAIL_ANGLE_MAX>` parameters should be set
to the angle to boom is to the centre line at each extreme of its travel. This allows the angle of the
boom to be calculated at any point between. 

The ideal sail angle to the wind :ref:`(SAIL_ANGLE_IDEAL)<SAIL_ANGLE_IDEAL>` should then be set. This defines the angle
between the boom and the wind direction as reported by the wind vane. An angle of zero here would result
in the boom staying parallel to the wind vane. The boom will keep this angle to the wind until it reaches
either its minimum or maximum limit. If the sails are too loose this number should be increased. This can
be tested in ACRO mode. 

Wing sail elevator
------------------
A wing sail may be actuated by a secondary control surface akin to a plane's elevator, the wing is free to rotate on a mast and actuated only by the control surface. This should be setup such that the servo mid point (``SERVOx_TRIM``) results in zero angle of attack on the main wing.
See `Karl's build <https://discuss.ardupilot.org/t/sailboat-support/32060/321>`__ . Use servo output function, ``SERVOx_FUNCTION``, 128 - Wing Sail Elevator.

The servo min (``SERVOx_MIN``) and max (``SERVOx_MAX``) end points should be set such that the main wing is held in a maximum lift angle of attack on both tacks.
If the wing sail is acting on the wrong tack the servo should be reversed with ``SERVOx_REVERSE``. The `SAIL_ANGLE_*` parameters are not use with wing sail elevators.

Direct wing sail actuation
--------------------------
Wing sail is directly driven by a servo, in this case use servo output function, ``SERVOx_FUNCTION``, 137 - Sail Mast Rotation. See `Ari's build <https://github.com/ArduPilot/ardupilot/pull/17066#issuecomment-813692335>`__ 

The servo mid point (``SERVOx_TRIM``) must be set such such that the wing is held on the center line of the boat. The min (``SERVOx_MIN``) and max points (``SERVOx_MAX``) should be set
such that the wing moves to the same angle on each tack. This maximum angle should be measured and set to the :ref:`SAIL_ANGLE_MAX<SAIL_ANGLE_MAX>` parameter. For best results this should be close to 90 deg. 
As with sheeted sails :ref:`SAIL_ANGLE_IDEAL<SAIL_ANGLE_IDEAL>` controls the angle the wing is held to the wind. There is not such visual feedback for under and over sheeting as there is with a soft sail so some testing might be required to find the best value.

Note that to stop the wing sail will be moved to the center line of the boat rather than actively driven to point into the wind.

Motoring
---------
Sailboat also supports motoring, this can be user selected or based on a wind speed threshold. The motor can take any form and should be setup as it would be for a normal rover.
In this case the throttle stick will control both the motor and the sail. To avoid this conflict sail control can be set to a second RC input channel by setting a `RCx_OPTION` to 207 - Mainsail.
The secondary channel will only be active in modes where the operator has direct manual control of the sail, in 'higher' control modes the main throttle channel will still be used.

If configured with both a motor and sail the vehicle will automatically switch based on the conditions. :ref:`SAIL_WNDSPD_MIN<SAIL_WNDSPD_MIN>` defines the wind speed threshold, in winds above this speed the vehicle will sail
tacking to its destination if required. When the wind speed decrees below that threshold the vehicle will drive to its destination on a direct path as a normal Rover would. If the vehicle gets stuck while trying to tack it will
also use its throttle to try and complete the tack. 

`RCx_OPTION`  74 - Sailboat motoring 3pos allows the operator to control the motoring states. The low position the motor is disabled, the vehicle will always try to sail. The middle position is the default automatic switching as explained above. 
The switch high position is motoring only, the vehicle will always use the motor and never try to sail.

.. note::
  when sailing there is no speed control, the vehicle is trying to sail or it releases the sail and tries to stop. When motoring speed control works as normal.
