.. _balance_bot-hardware: 

==============
Hardware Setup
==============

Look up the rover hardware :ref:`assembly<rover-assembly-instructions>` page for a step by step guide. This page only deals with the extra balance specific information.

Building your Balance Bot
-------------------------
Take a look at the :ref:`Arduroller<reference-frames-arduroller>` page for instructions on building your Balance Bot from scratch.

Supported Hardware:
-------------------
The Balance Bot requires wheel encoders to run most flight modes. These require 4 Auxilliary pins on the flight controller. This places certain limitations on the hardware we can use. 

Flight Controllers
==================

#. All  full sized **Pixhawk** variants(Pixfalcon and Pixhawk Mini lack aux pins)
#. **Pixracer** (2 pins for PWM, other 4 for wheel encoders)

Motor Drivers/ESC
=================
We strongly recommend using motor drivers or ESCs which take servo **PWM** (50Hz,1-2ms) as input, like this one from `Pololu <https://www.pololu.com/product/3284>`__.  Duty cycle based motor drivers and those which use relay pins to control direction are not compatible with the Balance Bot as of now.

Wheel Encoders
==============
The wheel encoder libraries are designed to run only with quadrature encoders with two output channels. 










