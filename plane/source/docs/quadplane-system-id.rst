.. _quadplane-system-id:

=============================
System Identification Feature
=============================

The System Identification feature is for advanced users and provides a means to characterize the response of a vehicle in the 
frequency domain.  The flight data collected from this mode can be used in several ways:

 - generating mathematical models of the vehicles flight behavior for model generation
 - investigating and predicting the quality of the vehicle tune
 - calculating flying quality metrics

It injects an input signal, called a chirp, at various points in the control loops.  The chirp is a constant amplitude 
oscillation that increases in frequency from a user defined minimum to maximum frequency, generally referred to as a 
frequency sweep.  The injection points are defined by the :ref:`SID_AXIS<SID_AXIS>` parameter and there are points in 
the Attitude and Position Controller.  For any injection points in the Attitude Controller, the user is required to fly 
the aircraft through an underlying STABILIZE like flight mode.  For any injection points in the Position Controller, the 
user controls are locked out and will not be able to affect the control of the vehicle.  The resulting flight data is 
logged for math analysis and model generation after the flight.



Enabling the System Identification Feature
==========================================

This feature is not automatically compiled into the stable version of firmware.  Use the `custom firmware webtool <https://custom.ardupilot.org>` to compile a version for your board. Select the "system ID support for quadplanes" under the Plane category.

Setup
======

System ID feature in quadplane is different than copter.  The system ID feature in copter is a separate flight mode.  In quadplane, system ID is run on top of either qstabilize, qhover, or qloiter flight modes.  The system ID feature is started and stopped using an Aux Switch.

Set an aux switch on the transmitter to have RCX_OPTION set to 184.
