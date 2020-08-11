.. _qstabilize-mode:

===============
QSTABILIZE Mode
===============

QSTABILIZE mode allows you to fly your vehicle manually, but self-levels
the roll and pitch axis.

.. tip::

   If you're learning to fly, try :ref:`QHOVER <qhover-mode>` or
   :ref:`QLOITER <qloiter-mode>` instead of
   QSTABILIZE. You'll have fewer crashes if you don't need to concentrate on
   too many controls at once.


.. warning::

   While QSTABILIZE mode does not necessarily require GPS, switching to QRTL or RTL in case of emergency does. Make sure you do have a reliable
   position estimate prior to arming, most commonly provided by 3D GPS fix with sufficient HDOP. This is required by default in the arming checks. It is highly recommended that these checks not be  disabled.

.. warning::

    Flying Quadplane backwards at speed or backwards into high wind is not recommended, since the Plane control surfaces will act backwards and could increase instability.


Overview
========

-  Pilot's roll and pitch input control the lean angle of the QuadPlane.
   When the pilot releases the roll and pitch sticks the vehicle
   automatically levels itself.
-  Pilot will need to regularly input roll and pitch commands to keep
   the vehicle in place as it is pushed around by the wind.
-  Pilot's yaw input controls the rate of change of the heading.  When
   the pilot releases the yaw stick the vehicle will maintain its
   current heading.
-  Pilot's throttle input controls the average motor speed meaning that
   constant adjustment of the throttle is required to maintain
   altitude.  If the pilot puts the throttle completely down the motors
   will go to their minimum rate (Q_M_SPIN_ARMED) and if the vehicle
   is flying it will lose attitude control and tumble.
-  The throttle sent to the motors is automatically adjusted based on
   the tilt angle of the vehicle (i.e. increased as the vehicle tilts
   over more) to reduce the compensation the pilot must do as the
   vehicle's attitude changes.

.. note::

   Always switch into a manual mode such as QSTABILIZE if the
   autopilot fails to control the vehicle. Maintaining control of your
   copter is your responsibility.



Common Problems
===============

-  QuadPlane flips immediately upon take-off.  This is usually caused
   by the motor order being incorrect or spinning in the wrong direction
   or using an incorrect propeller (clockwise vs counter-clockwise). 
   Check the rc connections for your autopilot.
-  QuadPlane wobbles on roll or pitch axis.  This usually means the Rate P
   values are incorrect.  See Copter Tuning section for some hints as to
   how to adjust the Q_A_xxx parameters.
-  QuadPlane wobbles when descending quickly.  This is caused by the copter
   falling through its own prop wash and is nearly impossible to  tune
   out although raising the Rate Roll/Pitch P values may help.
-  QuadPlane yaws right or left 15degrees on take-off.  Some motors may not
   be straight or the escs need calibration :ref:`QuadPlane ESC Calibration <QuadPlane-esc-calibration>`.
-  QuadPlane does not maintain altitude or does not stay perfectly still in
   the air.  As mentioned above this is a manual flight mode and
   requires constant control of the sticks to maintain altitude and
   position.
-  occasional twitches in roll or pitch.  Normally caused by some kind
   of interference on the receiver (for example FPV equipment placed too
   close to the receiver) or by ESC problems that may be resolved by
   :ref:`QuadPlane ESC Calibration <quadplane-esc-calibration>`.
-  sudden flips during flight.  This is nearly always caused by
   :ref:`mechanical failures <copter:common-diagnosing-problems-using-logs_mechanical_failures>`
   of the motor or ESCs.

Hover Throttle
==============

Usually, it is desired to hover in any mode at mid-stick on throttle, so that transitions between modes is easily accomplished without throttle position changes. This can be adjusted using the :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>` parameter, or automatically learned in QHOVER or QLOITER modes by enabling :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>`.

.. note:: If :ref:`Q_THROTTLE_EXPO<Q_THROTTLE_EXPO>` = 0 in QACRO and QSTABILIZE modes, then :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>`, whether set manually or learned via :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>` , is not applied, and the throttle is determined directly from the RC input.
