.. _px4-rover-build-with-brushless-traxxas-stampede-4wd-truck:

=========================================================
PX4 Rover Build with Brushless Traxxas Stampede 4WD Truck
=========================================================

This article explains how to add Rover running on a PX4 to the
`Brushless Traxxas Stampede four wheel drive truck <https://traxxas.com/products/models/electric/67086stampede4x4vxl>`__.

Overview
========

The *Brushless Traxxas Stampede four wheel drive truck* makes an
excellent choice for a Rover.

The truck's ESC provides forward and reverse so some special
considerations are necessary.

Wiring diagram
==============

The wiring diagram below illustrates a method for directly powering the
steering servo from the Stampede's ESC.

.. image:: ../../../images/PX4IOWiringRoverEscBec.jpg
    :target: ../_images/PX4IOWiringRoverEscBec.jpg

No solder connection of ESC and steering servo power
====================================================

The photograph below illustrates a no solder method for connecting the
ESC and steering servo power wire:

.. image:: ../../../images/RoverPX4ServoPower1.jpg
    :target: ../_images/RoverPX4ServoPower1.jpg

-  Remove the power pin sockets from the end of the connectors that plug
   into the PX4 on the servo and ESC cables.
-  Strip them loose of the 3 wire cables and connect them with a small
   piece of wire or (wire wrap pin).
-  Put heat shrink tubing over the joined sockets and shrink it on with
   a heat gun. (Shown before connection).

RC Setup
========

This truck uses the FRSky Delta 8 multi-protocol receiver which is
compatible with Hitec and Futaba RC transmitters. The Delta 8 provides
the required PPM-Sum communication with the PX4 using our own Futaba or
Hitec transmitter.

.. image:: ../../../images/RoverPX4Stampede2.jpg
    :target: ../_images/RoverPX4Stampede2.jpg


Mounting setup
==============

The 1/8" 6061 aluminum plate is excessive (and required heavier springs
on the suspension) but functions well.

-  In addition to the PX4 and receiver, the safety button and buzzer
   have also been mounted to the plate.
-  Slightly longer screws were used to install the plate and body
   mounting studs (a very simple installation).
-  The receiver is mounted with heavy duty Velcro and the PX4 will be
   remounted with Kyosho Zeal gel squares.

Cutouts for safety button access and the buzzer can be seen in the
truck's body below.

.. image:: ../../../images/rover_stampede_px4.jpg
    :target: ../_images/rover_stampede_px4.jpg

This truck has reverse and it transitions at mid throttle PWM so some
special setup and precautions are required.

-  I set up my Hitec Aurora 9 transmitter so throttle (channel 3) was
   mapped to my right (self centering) elevator stick.
-  And since I didn't want to steer on the same stick as my throttle I
   mapped steering (channel 1) to the left rudder stick.

Problems
========

There are still problems with this setup:

-  Currently throttle reversing is not directly supported and when
   anything goes wrong the PX4 sets throttle PWM to zero.
-  Of course this results in the truck being set to full speed in
   reverse.
-  I have to pick up the truck and unplug the battery when this occurs
   (a very unsatisfactory situation).
-  I am trying to get a firmware fix for this which resets to mid
   throttle position rather than zero.
-  I am also hoping to be able to get functional reversing made
   available in the auto modes as well.
