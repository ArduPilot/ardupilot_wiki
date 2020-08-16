.. _quadplane-flight-modes:

QuadPlane Flight modes
======================

The QuadPlane code is based upon the Plane master firmware but with 6
extra modes. Most are identical to equivalent Copter modes:

.. toctree::
    :maxdepth: 1

    QSTABILIZE (Mode 17) <qstabilize-mode>
    QHOVER    (Mode 18) <qhover-mode>
    QLOITER   (Mode 19) <qloiter-mode>
    QLAND     (Mode 20) <qland-mode>
    QRTL      (Mode 21) <qrtl-mode>
    QAUTOTUNE (Mode 22) <qautotune-mode>
    QACRO     (Mode 23) <qacro-mode>
    AIRMODE** <airmode>

** not actually a flight mode, but rather a feature of QACRO and QSTABILIZE

.. tip::

   You may need to set the ``FLTMODE*`` parameters for these
   extra modes as numeric values if your GCS doesn't understand these
   values yet.

If you are familiar with the equivalent Copter flight modes then you
should be comfortable flying a QuadPlane. The most significant difference comes
during transition between fixed wing and QuadPlane flight, which is
described :ref:`here <quadplane-flying>`.

.. tip::

   There is also a difference in throttle channel parameters from Copter: instead of :ref:`THR_DZ<THR_DZ>` for setting the deadzone, QuadPlane uses RCn_DZ (where n is the channel mapped to throttle input). 
   The default for throttle channel deadzone is 60 (+/- 6%).
   If your Quadplane does not hover at mid-stick (+/- 6%) in QSTABILIZE, you will need to center it using :ref:`Q_M_THST_HOVER<Q_M_THST_HOVER>` to set the throttle percentage applied at mid-stick in QSTABILIZE such that switching to QHOVER or QLOITER does not result in an altitude change. This value can be automatically learned and applied during QLOITER and QHOVER modes, by enabling :ref:`Q_M_HOVER_LEARN<Q_M_HOVER_LEARN>` .

.. tip::

   Since Quadplanes have much higher surface area than most Copters, hovering or loitering in a tail wind can be a challenge for stability. It is generally better to hover or loiter with the nose pointed into the wind, see :ref:`Weathervaning <quadplane-weathervaning>`.

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
