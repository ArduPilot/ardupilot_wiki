.. _rover-tuning-navigation:

=================
Tuning Navigation
=================

This page decribes how to tune the Navigation control including the "L1 controller".

.. image:: ../images/rover-tuning-navigation1.png
    :target: ../_images/rover-tuning-navigation1.png

L1 controller
-------------

The L1 one controller is the highest level steering controller.  It accepts position targets (i.e. latitude, longitude points) and the vehicle's current speed and outputs a desired lateral acceleration which is then fed into lower level controllers (including the Steering Rate controller).  Together this leads to the vehicle tracking a line towards the target.  This controller is used in all the autonomous modes including :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` and :ref:`SmartRTL <smartrtl-mode>`.

Recommended steps for tuning the L1 controller:

- connect the ground station to the vehicle using a telemetry radio
- create an rectangular or back-and-forth mission (perhaps using the Mission Planner's Flight Plan screen) and upload to the vehicle
- arm the vehicle (in :ref:`Manual <manual-mode>` or :ref:`Hold <hold-mode>`) and switch to :ref:`Auto <auto-mode>` mode.
- if the rover weaves along the straights, then raise :ref:`NAVL1_PERIOD <NAVL1_PERIOD>` (aka "Lat Acc Cntrl Period") in increments of 0.5
- if the rover does not turn sharply enough then lower :ref:`NAVL1_PERIOD <NAVL1_PERIOD>` (aka "Lat Acc Cntrl Period") in increments of 0.5
- raise :ref:`NAVL1_DAMPING <NAVL1_DAMPING>` (aka "Lat Acc Cntrl Damp") by increments of 0.05 to improve navigation in tight courses

.. image:: ../images/rover-tuning-navigation-mission.png
    :target: ../_images/rover-tuning-navigation-mission.png

Other Parameters
----------------

- :ref:`WP_SPEED <WP_SPEED>` defines the target speed (in m/s) that the vehicle should drive at in :ref:`Auto <auto-mode>` and :ref:`Guided <guided-mode>` (will also be used in :ref:`RTL <rtl-mode>` and :ref:`SmartRTL <smartrtl-mode>` if :ref:`RTL_SPEED <RTL_SPEED>` is set to zero)
- :ref:`TURN_MAX_G <TURN_MAX_G>` defines the maximum lateral acceleration (in Gs = 9.81m/s/s) the controllers will attempt during cornering.  If the vehicle leans dangerously or flips in corners this number should be reduced
- :ref:`WP_RADIUS <WP_RADIUS>` specifies the distance before the waypoint that the vehicle may begin turning towards the next waypoint
- :ref:`WP_OVERSHOOT <WP_OVERSHOOT>` is used to slow the vehicle before it reaches a waypoint.  It is the maximum distance (in meters) that the vehicle may drive beyond the waypoint as it turns towards the next waypoint
- :ref:`PIVOT_TURN_ANGLE <PIVOT_TURN_ANGLE>` is only used by skid steering vehicles.  If the vehicle's heading error becomes larger than this value (in degrees) the vehicle will momentarily stop and pivot towards the target before continuing.
