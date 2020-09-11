.. _flowhold-mode:

=============
FlowHold Mode
=============

FlowHold mode uses an :ref:`optical flow sensor <common-optical-flow-sensors-landingpage>` to hold position without the need for a GPS nor a downward facing Lidar.  Better performance can be achieved by attaching a :ref:`rangefinder <common-rangefinder-landingpage>` and using regular :ref:`Loiter <loiter-mode>` mode instead

..  youtube:: SSISkG58cDk?t=70
    :width: 100%

.. warning::

   With many vehicles this mode results in wobbling and unstable position hold.  If possible we recommend attaching a :ref:`rangefinder <common-rangefinder-landingpage>` and using regular :ref:`Loiter <loiter-mode>` instead

FlowHold mode is similar to PosHold mode in that the pilot directly controls the vehicle's lean angles with the roll and pitch sticks.  When the user releases the sticks, the optical flow sensor is used by the autopilot to bring the vehicle to a stop.

Because no Lidar is used in this mode, the optical flow sensor is used both to estimate the vehicle's height above the surface and the vehicle's speed.  Soon after takeoff or after there have been large changes in the height above the surface the vehicle may wobble as it learns the new height and velocity.

The following parameters can be used to tune FlowHold's performance:

-  :ref:`FHLD_BRAKE_RATE <FHLD_BRAKE_RATE>` controls the desired deceleration rate when the pilot releases the sticks
-  :ref:`FHLD_FILT_HZ <FHLD_FILT_HZ>` is the filter applied to the optical flow sensor data
-  :ref:`FHLD_FLOW_MAX <FHLD_FLOW_MAX>` is the optical flow sensor's maximum rate (in m/s) used when stopping
-  :ref:`FHLD_QUAL_MIN <FHLD_QUAL_MIN>` is the minimum flow quality used.  If the flow sensor quality falls below this value, FlowHold will act just like AltHold
-  :ref:`FHLD_XY_P <FHLD_XY_P>`, :ref:`FHLD_XY_I <FHLD_XY_I>`, :ref:`FHLD_XY_IMAX <FHLD_XY_IMAX>` and :ref:`FHLD_XY_FILT_HZ <FHLD_XY_FILT_HZ>` gains to tune the stopping response

