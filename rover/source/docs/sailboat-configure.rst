.. _sailboat-configure:

=======================
Configuration and Setup
=======================

All sailing parameters can be found by searching for the prefix  :ref:`SAIL <SAIL_ENABLE>`, the
sailing code relies heavily on getting correct information on the wind direction and strength from
the :ref:`wind vane <wind-vane>`.

Other useful parameters
------------------------
:ref:`SAIL_NO_GO_ANGLE <SAIL_NO_GO_ANGLE>` defines the no go zone into witch the sailing vehicle cannot
travel. In auto heading modes the vehicle will tack at this angle into the true wind If the desired heading
is within the no go zone. Note: this angle will be used whatever the wind strength and should be set with
that in mind.

``WP_OVERSHOOT`` defines the maximum cross track error in auto mode that is allowed before
the vehicle will tack. This keeps the vehicle within a corridor of width 2* :ref:`SAIL_XTRACK_MAX<SAIL_XTRACK_MAX>`. If set to zero
the vehicle will ignore the cross track error and only tack once it can reach its destination.

:ref:`PIVOT_TURN_RATE <PIVOT_TURN_RATE>` defines the maximum rate used for tacking, a lower value will result
in slower tack. This should be reduced if the vehicle is tacking too sharply and losing momentum while tacking.

:ref:`LOIT_RADIUS <LOIT_RADIUS>` defines the radius from the loiter point the vehicle will try and stay within,
the vehicle will keep moving and tack back towards the loiter point once it reaches this radius.

:ref:`SAIL_HEEL_MAX <SAIL_HEEL_MAX>` defines the angle at which the sail heel control PID controller is enabled.
If the heel is larger than this angle the PID controller will target this angle however if the heel is less the
controller will not try and reach it. i.e. only sheet out, don't sheet in.

Heel control PIDs values are set using ``ATC_SAIL_x`` parameters. The effect of changing the value
can be seen in ACRO mode by manually heeling the vehicle. Most control should be done using the P (:ref:`ATC_SAIL_P<ATC_SAIL_P>`) and I(:ref:`ATC_SAIL_I<ATC_SAIL_I>`)terms. D gain (:ref:`ATC_SAIL_D<ATC_SAIL_D>`)
is usually too quick for the relatively slow response of the sail winch servos so should be left at zero. The PID
values can be sent via Mavlink using :ref:`GCS_PID_MASK <GCS_PID_MASK>`.

Tacking in ACRO and AUTO mode while traveling upwind can be triggered through the use of an aux switch. This can
be setup by setting the ``RCx_OPTION`` parameter to function 63 – sailboat tack.

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

The heel angle controller can be setup in two ways. A low I term (:ref:`ATC_SAIL_I<ATC_SAIL_I>`) can be used with a low max heel angle. In
this case the controller will never hold at the max heel angle but will progressively sheet out as heel is
increased. Alternatively the heel angle (:ref:`SAIL_HEEL_MAX <SAIL_HEEL_MAX>`) can be set at the actual maximum desired heel and then higher gains
used to more aggressively let the sails out. In this case larger I and I max values should be used. Unlike
the P and I terms the D term is always active however due to the slow response of typical sail winch servos
it is unlikely to prove useful, vehicles with faster servos may benefit from a small amount of D gain.

Fences
------

Sailboats behave in the same manner as other Rovers regarding fence operation and breach failsafe actions. However, unlike other Rovers, which slow as they approach a fence boundary, Sailboats will just attempt to tack away from the boundary since they have no speed controller, as such.