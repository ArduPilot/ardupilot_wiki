.. _sailboat-configure:

=======================
Configuration and Setup
=======================

The vehicle steering output channel should be set to servo function 26 – Ground steering. The sail
output channel should be set to servo function 89 - Main Sail. 

All sailing parameters can be found by searching for the prefix  :ref:`SAIL <SAIL_ANGLE_MIN>`, the
sailing code relies heavily on getting correct information on the wind direction and strength from
the :ref:`wind vane <wind-vane>`.

Setting up sail range
---------------------
This can be tested by arming in manual mode; the throttle will directly control the sail position.
Throttle stick down (towards you) should result in the sail being sheeted in towards the centre line.
If the sail is sheeted out the servo should be reversed. The servo min and max parameters can then be
used to set up the range of travel. The min and max values should be set such that the boom is brought
in towards the centre line of the boat but not pulled down tightly. The boom should be able to be let
out until it reaches the shrouds; if no shrouds are fitted the boom shouldn’t go too far past 90 degrees
to the boat centre line. 

The sail angle :ref:`min <SAIL_ANGLE_MIN>` and :ref:`max <SAIL_ANGLE_MAX>` parameters should be set
to the angle to boom is to the centre line at each extreme of its travel. This allows the angle of the
boom to be calculated at any point between. 

The :ref:`ideal sail angle to the wind <SAIL_ANGLE_IDEAL>` should then be set. This defines the angle
between the boom and the wind direction as reported by the wind vane. An angle of zero here would result
in the boom staying parallel to the wind vane. The boom will keep this angle to the wind until it reaches
either its minimum or maximum limit. If the sails are too loose this number should be increased. This can
be tested in ACRO mode. 

Other usefull parameters
------------------------
:ref:`SAIL_NO_GO_ANGLE <SAIL_NO_GO_ANGLE>` defines the no go zone into witch the sailing vehicle cannot
travel. In auto heading modes the vehicle will tack at this angle into the true wind If the desired heading
is within the no go zone. Note: this angle will be used whatever the wind strength and should be set with
that in mind.

:ref:`WP_OVERSHOOT <WP_OVERSHOOT>` defines the maximum cross track error in auto mode that is allowed before
the vehicle will tack. This keeps the vehicle within a corridor of width 2* :ref:`SAIL_XTRACK_MAX<SAIL_XTRACK_MAX>` . If set to zero
the vehicle will ignore the cross track error and only tack once it can reach its destination.

:ref:`PIVOT_TURN_RATE <PIVOT_TURN_RATE>` defines the maximum rate used for tacking, a lower value will result
in slower tack. This should be reduced if the vehicle is tacking too sharply and losing momentum while tacking.

:ref:`LOIT_RADIUS <LOIT_RADIUS>` defines the radius from the loiter point the vehicle will try and stay within,
the vehicle will keep moving and tack back towards the loiter point once it reaches this radius.

:ref:`SAIL_HEEL_MAX <SAIL_HEEL_MAX>` defines the angle at which the sail heel control PID controller is enabled.
If the heel is larger than this angle the PID controller will target this angle however if the heel is less the
controller will not try and reach it. i.e. only sheet out, don't sheet in.

Heel control PIDs values are set using :ref:`ATC_SAIL <ATC_SAIL_P>` parameters. The effect of changing the value
can be seen in ACRO mode by manually heeling the vehicle. Most control should be do using the P and I terms. D gain
is usually too quick for the relatively slow response of the sail winch servos so should be left at zero. The PID
values can be sent via Mavlink using :ref:`GCS_PID_MASK <GCS_PID_MASK>`.

Tacking in ACRO and AUTO mode while traveling upwind can be triggered through the use of an aux switch. This can
be setup by setting the RCx_OPTION parameter to function 63 – sailboat tack.

Tuning
------

The steering rate and navigation should be tuned in the usual way. Care should be taken that the final parameters
work well on all points of sail and at range of wind speeds. For tuning the navigation controller it is
recommended that a simple two point mission is run. The mission should be set up such that the boat travels at
90 degrees to the wind. This can be run indefinitely using a do_jump waypoint. Note that the L1 controller is
only used when the vehicle is not tacking close to the wind.

If the vehicle aggressively responds to changes in wind direction when traveling upwind either the wind vane
direction filter frequency can be reduced or the maximum straight line rate reduced. Note that reducing the
filter frequency will also slow the response of the sails.

The heel angle controller can be setup in two ways. A low I term can be used with a low max heel angle. In
this case the controller will never hold at the max heel angle but will progressively sheet out as heel is
increased. Alternatively the heel angle can be set at the actual maximum desired heel and then higher gains
used to more aggressively let the sails out. In this case larger I and I max values should be used. Unlike
the P and I terms the D term is always active however due to the slow response of typical sail winch servos
it is unlikely to prove useful, vehicles with faster servos may benefit from a small amount of D gain.
