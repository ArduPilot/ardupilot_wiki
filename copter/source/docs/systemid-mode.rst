.. _systemid-mode:

==========================
System Identification Mode
==========================

This mode is for advanced users and provides a means to characterize the response of a vehicle in the frequency domain.  The flight data collected from this mode can be used in several ways:
 - generating mathematical models of the vehicles flight behavior for model generation
 - investigating and predicting the quality of the vehicle tune
 - calculating flying quality metrics
 
It injects an input signal, called a chirp, at various points in the control loops.  The chirp is a constant amplitude oscillation that increases in frequency from a user defined minimum to maximum frequency, generally referred to as a frequency sweep.  The injection points are defined by the :ref:`SID_AXIS<SID_AXIS>` parameter and there are points in the Attitude and Position Controller.  For any injection points in the Attitude Controller, the user is required to fly the aircraft through an underlying STABILIZE like flight mode.  For any injection points in the Position Controller, the user controls are locked out and will not be able to affect the control of the vehicle.  The resulting flight data is logged for math analysis and model generation after the flight.


Enabling the Mode
=================

If the :ref:`SID_AXIS<SID_AXIS>` is non-zero, then this modes associated parameters will become visible on the next parameter refresh and entry into the mode will be allowed. Otherwise, an error message will be logged and sent to the ground control station indicating that mode entry has been prohibited.

Further Information
===================

Information on this modes parameters and operation are given in the links below:

.. toctree::
    :maxdepth: 1

    System ID Mode Operation <systemid-mode-operation>