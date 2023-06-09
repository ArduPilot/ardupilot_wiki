.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.5 release

[copywiki destination="plane,copter,rover,blimp"]

on :ref:`common-buzzer` page, add the following to the buzzer sounds table:

- Gryo Init complete    7 short beeps
- Ready to ARM          beep-beep-beep-beeeeeep

[site wiki="plane"]

on :ref:`arming-your-plane` page, under "How to Arm":
=====================================================

add note to Rudder Arming:

.. note:: when rudder arming in QuadPlanes with an autotakeoff, the motors will spin at :ref:`Q_M_SPIN_ARM<Q_M_SPIN_ARM>` and not takeoff until the rudder stick is returned to neutral. Similarly, for normal plane MODE TAKEOF, or autotakeoffs, the arming will not actually occur until the rudder stick is returned to neutral to prevent the takeoff starting with full right rudder.

`Takeoff Mode enhancement <https://github.com/ArduPilot/ardupilot_wiki/pull/5173>`__

[/site]
