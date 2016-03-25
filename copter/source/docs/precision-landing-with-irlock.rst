.. _precision-landing-with-irlock:

==============================
Precision Landing with IR-LOCK
==============================

This article shows how to enable precision landing on Copter using the
IR-LOCK sensor.

.. note::

   This feature is (will be) supported from Copter 3.4

Overview
========

Copter 3.4 (not yet released) supports precision landing using the
IR-LOCK sensor. Using this system, it is possible to land within 30cm of
an IR beacon that is moving at less than 1m/s.

..  youtube:: rGFO73ZxADY
    :width: 100%

Where to get it
===============

The `IR-LOCK sensor <http://irlock.com/collections/frontpage/products/ir-lock-sensor-precision-landing-kit>`__
can be purchased from `irlock.com <http://irlock.com/>`__.  The IR-LOCK
sensor is a modified version of the `Pixy camera <http://charmedlabs.com/default/pixy-cmucam5/>`__, which comes
pre-configured to work as an IR beacon detector. There are multiple IR
beacons which are compatible with the sensor. The `MarkOne Beacon <http://irlock.com/collections/shop/products/markone-beacon>`__
can be reliably detected in **all** **lighting conditions**, with a
detection range of **15 meters**. `Beacon (V1.1) <http://irlock.com/collections/shop/products/beacon>`__ is a more
cost-effective option which can be reliably detected in **most lighting
conditions**.

.. figure:: ../images/sensorandMarkers01.jpg
   :target: ../_images/sensorandMarkers01.jpg

   IR-LOCK Sensor and IR Beacons

Connecting to Pixhawk
=====================

The IR-LOCK sensor can be connected directly to Pixhawk via an `I2C cable <http://irlock.com/collections/shop/products/pixhawk-cable>`__. If
you are using multiple I2C sensors, then you will need an \ `I2C splitter <http://store.jdrones.com/Pixhawk_I2C_splitter_p/dstpx4i2c01.htm>`__.
More detailed instructions are included in the `irlock.com Documentation <https://irlock.readme.io/docs>`__. The IR-LOCK sensor can
also be `connected via USB to a Linux system <https://irlock.readme.io/docs/interfacing-sensor-w-linux-and-python>`__,
and sensor output can be retrieved in Python.

.. figure:: ../images/precision_landing_connect_irlock_to_pixhawk.jpg
   :target: ../_images/precision_landing_connect_irlock_to_pixhawk.jpg

   IRLock sensor/Pixhawk Wiring

Building the firmware
=====================

Since precision landing is not yet a default feature, you must use a
`pre-compiled firmware <https://irlock.readme.io/docs/ac33-precision-landing-firmware>`__
provided by IR-LOCK, or the :ref:`firmware must be re-built <dev:building-the-code>` with the
precision landing feature enabled. If you are re-building, make sure
that \ `this line <https://github.com/diydrones/ardupilot/blob/master/ArduCopter/APM_Config.h#L41>`__
in APM_Config.h is uncommented to look like below.

``#define PRECISION_LANDING ENABLED``

Mounting to the frame
=====================

The IRLOCK sensor should be mounted to the underside of the frame with
the camera lens pointing directly down toward the ground.  A mounting
bracket for IRIS is sold
`here <http://irlock.com/collections/frontpage/products/sensor-bracket-for-iris>`__
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
to enable the precision landing feature.

-  :ref:`PLND_ENABLED <PLND_ENABLED>` 1
-  :ref:`PLND_TYPE <PLND_TYPE>` 2

.. note::

   Remember to reboot the Pixhawk after making these changes.

Flying and Testing
==================

Setup the vehicle with one of the flight modes set to LAND (as of the
time this wiki page was written, the precision landing function only
operates in LAND mode).

Place the IR beacon on the ground and take-off to approximately 10m
above the target.  Switch the vehicle to LAND.  If everything is working
properly, the copter should move toward the IR beacon.  A successful
demo is shown below (using an older APM firmware).

.. tip::

   Be prepared to retake control if there are sudden unexpected
   movements (Change mode to Stabilize, AltHold or Loiter).

If the vehicle does behave appropriately, download the dataflash logs
and examine the PL messages.

-  If the "Heal" field is not "1" then there may be a communication
   issue between the Pixhawk and IR-LOCK sensor
-  If the eX/eY values do not appear 'smooth' then the sensor may be
   picking up false targets.  Refer to the IR-LOCK `wiki page <https://irlock.readme.io/docs/interpreting-pl-logs>`__ for more
   trouble-shooting information.

..  youtube:: IRfo5GcHniU
    :width: 100%
