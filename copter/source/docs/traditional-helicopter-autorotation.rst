.. _traditional-helicopter-autorotation:

============
Autorotation
============

Practice (Manual) Autorotation Setup
====================================

In versions 4.4 and later, the ability to conduct power re-engagement from an autorotation has been added.  This feature will work with ESC's using their built-in governor, the ardupilot throttle curve or the ardupilot built-in governor.  Three parameters were added to accommodate this feature.

* :ref:`H_RSC_AROT_MN_EN <H_RSC_AROT_MN_EN>` - Enables manual autorotations (0 - Disable, 1 - Enable)
* :ref:`H_RSC_AROT_ENG_T <H_RSC_AROT_ENG_T>` - Time to ramp throttle back to motorlock enabled value during power recovery in seconds
* :ref:`H_RSC_AROT_IDLE <H_RSC_AROT_IDLE>` - Idle throttle value in percent used during autorotation.  This can be set to the value that signals the ESC to perform a fast spool-up when performing a power recovery.

ESC Governor
------------

Enable manual autoratation by setting :ref:`H_RSC_AROT_MN_EN <H_RSC_AROT_MN_EN>` to 1.  Leave :ref:`H_RSC_AROT_ENG_T <H_RSC_AROT_ENG_T>` at the default setting of 1 second to quickly signal the ESC for fast spool-up.  Set the throttle idle value, :ref:`H_RSC_AROT_IDLE <H_RSC_AROT_IDLE>`, to the percent of throttle given by the ESC manual that signals the ESC to perform a fast spool-up.

Ardupilot's Throttle Curve or Built-in Governor
-----------------------------------------------

Enable manual autoratation by setting :ref:`H_RSC_AROT_MN_EN <H_RSC_AROT_MN_EN>` to 1.  Set :ref:`H_RSC_AROT_ENG_T <H_RSC_AROT_ENG_T>` to bring the throttle back to the throttle curve quickly.  One to two seconds woudld be sufficient.  If desired, the throttle idle value, :ref:`H_RSC_AROT_IDLE <H_RSC_AROT_IDLE>`, can be set to a higher value for ICE engines to remain at a higher engine RPM in the descent.  For electric engines using the throttle curve or built-in governor, set it to zero.

Semi-Autonomous Autorotation Setup
==================================

Future enhancement to provide autonomous pitch control during autorotation is planned.