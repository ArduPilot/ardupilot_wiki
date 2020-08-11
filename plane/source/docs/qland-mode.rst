.. _qland-mode:

==========
QLAND Mode
==========

QLAND Mode attempts to bring the QuadPlane straight down at the position the vehicle is located when the mode is entered, descending to :ref:`Q_LAND_FINAL_ALT <Q_LAND_FINAL_ALT>` at :ref:`Q_WP_SPEED_DN <Q_WP_SPEED_DN>` until it reaches :ref:`Q_LAND_FINAL_ALT <Q_LAND_FINAL_ALT>`, at which point it continues to descend at :ref:`Q_LAND_SPEED <Q_LAND_SPEED>` until landing.


.. note::

    QuadPlane will recognize that it has landed if the motors are at
    minimum but its climb rate remains between -20cm/s and +20cm/s for one
    second.  It does not use the altitude to decide whether to shut off the
    motors except that the QuadPlane must also be below 10m above the home
    altitude.

-  If the QuadPlane appears to bounce or balloon back up a couple of times
   before settling down and turning the props off, try lowering the
   ``Q_LAND_SPEED`` parameter a bit.
-  If the vehicle has GPS lock the landing controller will attempt to
   control its horizontal position but the pilot can adjust the target
   horizontal position just as in QLOITER mode.
-  If the vehicle does not have GPS lock the horizontal control will be
   as in QSTABILIZE mode so the pilot can control the roll and pitch lean
   angle of the QuadPlane.


.. warning::

    In any mode based on using the barometer: QLAND, QLOITER, QHOVER, QRTL, if your QuadPlanes     operation becomes erratic when you
    are close to the ground or landing (and also if any auto landing
    procedure results in bouncing or failure to turn off motors properly
    after landing) you probably have the autopilot situated such that
    its barometer (altimeter) is being affected by the pressure created by
    the QuadPlanes prop-wash against the ground.



-  This is easily verified by looking at the altimeter (baro alt) reading in your
   logs and seeing if it spikes or oscillates when near the ground.
-  If this is a problem, move the autopilot out of prop wash
   effect or shield it with an appropriately ventilated enclosure.
-  Success can be verified by flight test and by log results.

