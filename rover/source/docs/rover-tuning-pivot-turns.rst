.. _rover-tuning-pivot-turns:

==================
Tuning Pivot Turns
==================

..  youtube:: NNmGw4cDsB0
    :width: 100%

This page decribes how to tune the "pivot turn" feature on a Rover (or boat) with :ref:`skid-steering <rover-motor-and-servo-connections_skid-steering>`.

A pivot turn involves the vehicle slowing or stopping, turning towards the destination and then continuing.  These turns are only triggered in :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` or :ref:`SmartRTL <smartrtl-mode>` modes.

Parmeters
---------

- :ref:`PIVOT_TURN_ANGLE <PIVOT_TURN_ANGLE>` holds the minimum angle error that will trigger the pivot turn.  For example when set to the default of "60", a pivot turn will be triggered whenever the vehicle's heading is at least 60 degrees off from the next waypoint.  This parameter should normally not be set below 30 unless you wish to disable pivot-turns which can be done by setting this parameter to 0.  The vehicle will begin towards the next waypoints once its heading is within 10 degrees of the heading to the destination
- :ref:`ATC_STR_ANG_P <ATC_STR_ANG_P>` converts the vehicle's heading error into a desired turn rate.  Higher values lead to a more aggressive turn towards the destination.
- :ref:`ATC_STR_RAT_MAX <ATC_STR_RAT_MAX>` limits the vehicle's maximum turn rate (in degrees per second) in all modes including during pivot turns.
- :ref:`ATC_STR_ACC_MAX <ATC_STR_ACC_MAX>` limits the vehicle's maximum rotational acceleration (in deg/sec/sec) in all modes.  higher numbers lead to the vehicle getting up to its maximum turn rate more quickly

Videos
------

..  youtube::  G6BuZuNi5Og   
    :width: 100%

*box boat performing pivot turns*