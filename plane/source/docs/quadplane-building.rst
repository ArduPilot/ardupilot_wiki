.. _quadplane-building:

Building a QuadPlane
====================

Putting together a QuadPlane involves careful planning. This page will
give you some general guidance on design principles for QuadPlanes
that may help you with your build.

.. image:: ../images/quadplane-porter-octaquad.jpg
    :target: ../_images/quadplane-porter-octaquad.jpg

General Rules
-------------

A wide range of fixed wing aircraft can be converted to have VTOL
capabilities. While ArduPilot uses the name QuadPlane for these
aircraft, you are not restricted to just QuadCopter motor
layouts. Almost any multicopter motor arrangement can be used with a
QuadPlane, including quad, hexa, tricoper, octa and octaquad. In addition,
tilting motors to allow their use in both VTOL and Fixed Wing flight is
possible. Finally, versions which use a vertical stance (nose vertical) for
VTOL operations are possible.

Some of the key factors to success are:

- a fixed wing frame that can carry the weight of the additional
  lifting motors and power system, along with any payload needed

- sufficient power in the lifting motors not just for the total
  airframe weight, but also for the additional load that may be
  induced by downforce on the wings for configurations using a horizontal VTOL stance.

- complete clearance above and below the whole disk area of the
  lifting motors, to ensure they achieve full aerodynamic thrust

- minimum wing, frame, and mount twist and flex so the motors provide thrust vertically
  at all times

- a sufficiently robust mounting system for the lifting motors

- minimizing aerodynamic drag from the lifting motors and frame

  .. image:: ../images/quadplane-quadstar.jpg
    :target: ../_images/quadplane-quadstar.jpg

When you are designing a QuadPlane it is highly recommended that you
make use of `eCalc <http://ecalc.ch/>`__ to help choose the motors,
ESCs, batteries, propellers and other components of your
design. Brushless motors vary a lot in their power to weight ratio,
and ensuring you choose motors that keep the weight down while
supplying sufficient lifting power is important.

 .. Tip:: Due to their greater mass and surface area, most QuadPlanes do not have as robust YAW authority as compared to small quadcopters (unless vectored thrust is employed with tilt rotors). It is extremely important that the motors be very well aligned, as just a few degress offset can sometimes effectively eliminate the ability to yaw in one or both directions. In fact, this effect can be used to actually increase yaw authority by purposely tilting one or both pairs of adjacent rotating motors in the natural direction of their torque 1 or 2 degrees (inward for H frame, outward for X and + frames).

QuadPlane Range
---------------

Perhaps surprisingly, it is sometimes possible to increase the
potential range of an aircraft using a QuadPlane conversion. This may
seem counter intuitive as a QuadPlane conversion will both add weight
and increase aerodynamic drag to an airframe.

The reason why range can be increased is the extra carrying capacity
of a QuadPlane. Many fixed wing aircraft are limited in the amount of
battery they can carry due to the requirements for reliable
launch. During launch, and especially when using a flying launch such
as a catapult or bungee, the aircraft needs to rapidly accelerate to
an airspeed above its stall speed. If it fails to reach that speed
suffiently quickly then it will crash. A QuadPlane avoids this problem
by taking off vertically, and can spend longer on the acceleration
needed to sufficient speed for forward flight.

This means it is often possible to pack a lot more battery into a
QuadPlane than is possible in the same airframe without VTOL
motors. The extra battery capacity can more than make up for the
increased weight and drag of the VTOL motors.

To make the most of this advantage you need to do very rapid VTOL
takeoffs and landings to minimise the battery consumption in VTOL
flight. The video below demonstrates just how rapid a takeoff can be
achieved with a properly setup quadplane.

..  youtube:: 4oVlSQaplZc
    :width: 100%
            
A second factor that can help with QuadPlane range is the flexibility
available in choosing the propeller and power train for the forward
motor. As conventional takeoff is not needed the forward motor does
not need to be optimised for the high level of thrust needed for
takeoff. This can allow larger propellers and geared motors to be used
that are highly efficient for forward cruise flight.

Finally, for really long range with a QuadPlane you can use an
internal combustion engine for the forward motor. A gas engine can
run for a lot longer than an electric motor with the same weight of
fuel.

Build Logs
----------

Here are some build logs of a few QuadPlanes that may help you with
ideas for your own build.

-  Porter OctaQuadPlane build:
   https://diydrones.com/profiles/blogs/building-flying-and-not-crashing-a-large-octaquadplane
-  Porter QuadPlane build:
   https://diydrones.com/profiles/blogs/building-flying-and-crashing-a-large-quadplane
-  The PerthUAV Mozzie build: http://mozzie.readthedocs.io/
-  Convergence conversion to ArduPlane Tilt Rotor Tricopter: https://discuss.ardupilot.org/t/e-flite-convergence-using-matek-f405-wing
-  C1 Chaser conversion to TVBS (Twin motor Vectored Belly Sitter, a Tailsitter that can takeoff and land from a plane stance): https://discuss.ardupilot.org/t/c1-chaser-tvbs-conversion-instructions
-  Foamboard Copter Tailsitter: https://discuss.ardupilot.org/t/box-kite-copter-tailsitter-build-instructions


if you would like to add your own build to this list then please
contact the ArduPilot dev team.
