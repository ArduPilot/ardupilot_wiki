.. _common-relay:

============
Relay Switch
============

A "Relay" is an digital output pin on the flight controller that can be switched between 0 volts and either 3.3V or 5V.  Similar to a servo it allows the flight controller to invoke some action from another device on the vehicle.  Up to 4 pins can be defined as relays.

Relay pins on the Pixhawk
=========================

On the Pixhawk AUX OUT 5 (pin 54) and AUX OUT 6 (pin 55) are setup as the "First" (or #0) and "Second" (or #1) Relays respectively.

.. image:: ../../../images/Relay_Pixhawk.jpg
    :target: ../_images/Relay_Pixhawk.jpg

The number of available Relays can be increased to a maximum of 6 by reducing the number of AUX pins used as :ref:`Servo <common-servo>` outputs.  This can be accomplished by reducing the :ref:`BRD_PWM_COUNT <BRD_PWM_COUNT>` from 4 to 2 or 0.

Defining the relay pins through the Mission Planner
===================================================

The First ~ Fourth relay pins can be defined most easily using the Mission Planner's Config/Tuning screen's Standard Params list as shown below.  Alternatively if you know the exactly pin number to use, the :ref:`RELAY_PIN <RELAY_PIN>` parameter can be set directly.

.. image:: ../../../images/Relay_SetupWithMP.png
    :target: ../_images/Relay_SetupWithMP.png

Pilot control of the relay
==========================

The relays can be controlled with the auxiliary switches.

.. image:: ../../../images/Relay_SetupCh7WithMP.png
    :target: ../_images/Relay_SetupCh7WithMP.png

Mission control of the relay
============================

Similar to a servo, the Relays can be activated during a mission using
the Do-Set-Relay mission command.  This is described on the :ref:`Copter Mission Command List wiki page <copter:mission-command-list_do-set-relay>`.
