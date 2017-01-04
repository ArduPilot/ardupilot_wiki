.. _failsafe-battery:

================
Battery Failsafe
================

This article explains how to setup and test the battery failsafe.

Overview
========

The battery failsafe can be set-up to automatically trigger an RTL or
LAND when the vehicle battery voltage or estimated remaining power has
crossed a configurable threshold.

To use this failsafe the vehicle must have a :ref:`Power Module or other voltage and (optionally) current monitor <common-powermodule-landingpage>`.

.. note::

   Copter also supports other failsafes: :ref:`Radio <radio-failsafe>`,
   :ref:`GCS <gcs-failsafe>`, and :ref:`EKF / DCM Check <ekf-inav-failsafe>`.

When the failsafe will trigger
==============================

If enabled and set-up correctly the battery failsafe will trigger if the
main battery:

-  voltage drops below 10.5 volts (configurable) for more than 10
   seconds
-  remaining capacity falls below the configurable Reserved MAH

What will happen
================

When the failsafe is triggered one of the following will happen:

-  **Nothing** if the vehicle is already disarmed
-  **Disarm motors** if the vehicle is in Stabilize or Acro mode and the
   throttle is at zero OR the vehicle is landed
-  **Return-to-Launch (RTL)** if the ``FS_BATT_ENABLE`` param is set to
   "2" ("RTL") OR the vehicle is in AUTO mode, has a GPS lock and are at
   least 2 meters from your home position
-  **Land** in all other cases

Setting up the battery failsafe
===============================

-  Setup the power module as described in the :ref:`Power Modules section of this wiki <common-powermodule-landingpage>` including setting the
   totally battery capacity if using a current monitor
-  On the **INITIAL SETUP \| Mandatory Hardware \| Failsafe** page:

   -  set the "Low Battery" threshold voltage (i.e. 10.5 volts)
   -  set the "Reserved MAH" or leave as "0" if the failsafe should
      never trigger based on estimated current consumed.  "600" would be
      an appropriate number to cause the vehicle to LAND or RTL when
      only 20% of a 3000mAH  battery remains.
   -  select the desired behavior as "Land" or "RTL" (Note: the
      placement of the drop-down is confusing but this will be corrected
      in a future version of the mission planner)

.. image:: ../images/Failsafe_Battery_Setup.png
    :target: ../_images/Failsafe_Battery_Setup.png

Low Battery warning (even with failsafe disabled)
=================================================

Even if the battery failsafe is not enabled (i.e.
``FS_BATT_ENABLE = 0``) the "Low Battery" voltage threshold and
"Reserved MAH" numbers are used to issue a battery warning. This will
result in:

-  "Low Battery!" message appearing on the ground stations HUD (if using
   telemetry)
-  Loud beeping if a buzzer is attached
-  Flashing LEDs

The only way to disable this warning completely is to set the "Low
Battery" voltage and "Reserved MAH" values to zero.
