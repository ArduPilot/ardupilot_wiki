.. _precision-landing-with-irlock:

=========================================
Precision Landing and Loiter with IR-LOCK
=========================================

Overview
========

Copter 3.4 (and higher) supports Precision Landing using the `IR-LOCK sensor <https://irlock.com/collections/frontpage/products/ir-lock-sensor-precision-landing-kit>`__ and a :ref:`sonar or lidar <common-rangefinder-landingpage>`.
Using this system, when the vehicle enters LAND mode (and has GPS lock) it is possible to reliably land within 30cm of an IR beacon that is moving at less than 1m/s.

Copter 3.5 (and higher) additionally supports Precision Loiter which allows a vehicle to maintain its position above a target while in Loiter mode.  The Pilot can enable this using one of the transmitter's :ref:`auxiliary function switches <common-auxiliary-functions>` (in versions prior to Copter-4.0, CH7_OPT or CH_8_OPT would be used).

..  youtube:: rGFO73ZxADY
    :width: 100%

Where to get it
===============

The `IR-LOCK sensor <https://irlock.com/collections/frontpage/products/ir-lock-sensor-precision-landing-kit>`__
can be purchased from `irlock.com <https://irlock.com/>`__.  The IR-LOCK
sensor is a modified version of the `Pixy camera <https://pixycam.com/pixy-cmucam5/>`__, which comes
pre-configured to work as an IR beacon detector. There are multiple IR
beacons which are compatible with the sensor. The `MarkOne Beacon <https://irlock.com/collections/markone>`__
can be reliably detected in **all** **lighting conditions**, with a
detection range of **15 meters**. `Beacon (V1.1) <https://irlock.com/collections/shop/products/beacon>`__ is a more
cost-effective option which can be reliably detected in **most lighting
conditions**.

.. figure:: ../images/sensorandMarkers01.jpg
   :target: ../_images/sensorandMarkers01.jpg

   IR-LOCK Sensor and IR Beacons

Connecting to Pixhawk
=====================

The IR-LOCK sensor can be connected directly to Pixhawk via an `I2C cable <https://irlock.com/collections/shop/products/pixhawk-cable>`__. If
you are using multiple I2C sensors, then you will need an \ `I2C splitter <http://store.jdrones.com/Pixhawk_I2C_splitter_p/dstpx4i2c01.htm>`__.
More detailed instructions are included in the `irlock.com Documentation <https://irlock.readme.io/docs>`__. 

.. figure:: ../images/precision_landing_connect_irlock_to_pixhawk.jpg
   :target: ../_images/precision_landing_connect_irlock_to_pixhawk.jpg

   IRLock sensor/Pixhawk Wiring

Mounting to the frame
=====================

The IRLOCK sensor should be mounted to the underside of the frame with
the camera lens pointing directly down toward the ground.  A mounting
bracket for IRIS is sold
`here <https://irlock.com/collections/frontpage/products/sensor-bracket-for-iris>`__
(and pictured below).  The sensor board should be oriented so that the
white button on the board points towards the front of the vehicle (or to
put it another way, the side closest to the camera lens should be
towards the front of the vehicle).

The image below shows the camera mounted on the *bottom* of a 3DR
IRIS+. It is probably best to mount the sensor as close as possible to
the Pixhawk, but successful tests have also been performed with
various mounting locations.

.. figure:: ../images/IRISbracket03.jpg
   :target: ../_images/IRISbracket03.jpg

   IR-LOCK Sensor Mounted onBottom of Iris+

..  youtube:: I8QF313F3bs
    :width: 100%

Setup through Mission Planner
=============================

Set the following parameters through the Mission Planner (or other GCS)
to enable the precision landing feature and then Reboot the autopilot.

-  :ref:`PLND_ENABLED <PLND_ENABLED>` = 1
-  :ref:`PLND_TYPE <PLND_TYPE>` = 2

To enable Precision Loiter, an :ref:`Auxiliary Function Switch <common-auxiliary-functions>` must be set to 39 to enable "Precision Loiter"

-  in versions prior to  Copter-4.0, a CHx_OPT parameter could be set via Mision Planner to 39 for this enable.

Flying and Testing
==================

Setup the vehicle with one of the flight modes set to LAND (as of the
time this wiki page was written, the precision landing function only
operates in LAND mode).

Place the IR beacon on the ground and take-off to approximately 10m
above the target.  Switch the vehicle to LAND.  If everything is working
properly, the copter should move toward the IR beacon.  A successful
demo is shown below (using an older firmware).

.. tip::

   Be prepared to retake control if there are sudden unexpected
   movements (Change mode to Stabilize, AltHold or Loiter).

If the vehicle does behave appropriately, download the dataflash logs
and examine the PL messages.

-  If the "Heal" (meaining health) field is not "1" then there may be a communication issue between the Pixhawk and IR-LOCK sensor.
-  If the "TAcq" (meaning Target Acquired) field is not "1" then the sensor is not seeing the target.
-  The pX, pY values show the horizontal distance to the target from the vehicle.
-  The vX, vY values show the estimated velocity of the target relative to the vehicle.

..  youtube:: IRfo5GcHniU
    :width: 100%

Precision Loiter demonstration:

..  youtube:: KoLZpSZDfII
    :width: 100%
