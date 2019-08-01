.. _quadplane-flight-modes:

QuadPlane Flight modes
======================

The QuadPlane code is based upon the Plane master firmware but with 5
extra modes:

-  mode 17: QSTABILIZE (like :ref:`Copter STABILIZE <copter:stabilize-mode>`)
-  mode 18: QHOVER (like :ref:`Copter ALT_HOLD <copter:altholdmode>`)
-  mode 19: QLOITER (like :ref:`Copter LOITER <copter:loiter-mode>`)
-  mode 20: QLAND (like :ref:`Copter LAND <copter:land-mode>`)
-  mode 21: QRTL (like :ref:`Copter RTL <copter:rtl-mode>`)
-  mode 23::ref:`QACRO <plane:qacro-mode>`

.. tip::

   You may need to set the ``FLTMODE*`` parameters for these
   extra modes as numeric values if your GCS doesn't understand these
   values yet.

If you are familiar with the equivalent Copter flight modes then you
should be comfortable flying a QuadPlane. The only real difference comes
during :ref:`Transistion<plane:quadplane-flying>` between fixed wing and QuadPlane flight and
QRTL which will simultaneously head to home while climbing or descending towards Q_RTL_ALT at home, instead of climbing first like Copter, and will then will begin descent immediately upon reaching home position.

.. tip::

   There is also a difference in throttle channel parameters from Copter: instead of THR_DZ for setting the deadzone, QuadPlane uses RCn_DZ (where n is the channel mapped to throttle input). 
   The default for throttle channel deadzone is 60 (+/- 6%).
   Also, there is no mapping of hover throttle to mid-stick in manual throttle modes. 
   If your quadplane does not hover at mid-stick (+/- 6%) in QSTABILIZE, you will need to center it (move it into the deadzone) when switching from QSTABILIZE to QHOVER to hold the current altitude.

.. tip::

   Since Quadplanes have much higher surface area than most Copters, hovering or loitering in a tail wind can be a challenge for stability. It is generally better to hover or loiter with the nose pointed into the wind, see Weathervaning <http://ardupilot.org/plane/docs/quadplane-weathervaning.html>.

.. note::

   The landing detection logic in QLOITER mode is not as sophisticated as the landing detection logic in Copter, so if you get GPS movement while on the ground in QLOITER mode then the aircraft may try to tip over as it tries to hold position while in contact with the ground. 
   It is suggested that you switch to QHOVER or QSTABILIZE once landed as these are unaffected by GPS movement.

Flight modes to avoid
---------------------

The linked nature of of the vertical lift and fixed wing control in quadplanes means the autopilot always needs to know what the pilot is trying to do in terms of speed and attitude. 
For this reason you should avoid the following flight modes in a quadplane:

-  ACRO
-  STABILIZE
-  TRAINING

these modes are problematic as the stick input from the pilot is not sufficient to tell the autopilot what attitude the aircraft wants or what climb rate is wanted, so the quadplane logic does not engage the quad motors when in these modes. 
These modes also make log analysis difficult. Please use FBWA mode instead of STABILIZE for manual flight.

In the future we may adds ways to use the quad motors in these modes, but for now please avoid them.

The other mode where the quad motors are disabled is MANUAL mode.
That mode still can be useful for checking aircraft trim in fixed wing flight or for taxiing your aircraft.

.. note:: If TILT_MASK is non-zero (:ref:`Tailsitter <plane:guide-tailsitter>` configuration), then some quad motors will be active in MANUAL mode since they are also used for forward propulsion.
