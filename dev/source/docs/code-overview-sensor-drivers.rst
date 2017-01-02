.. _code-overview-sensor-drivers:

==============
Sensor Drivers
==============

ArduPilot supports a wide variety of sensors from many different manufacturers.  One clear example of this can be seen in the :ref:`list of range finders <copter:common-rangefinder-landingpage>` (aka sonars, lidars).
This page attempts to explain how sensor drivers are written and integrated into the vehicle code.

FrontEnd / BackEnd Split
========================

.. image:: ../images/code-overview-sensor-drivers-febesplit.png

One important concept within the sensor driver architecture is the front-end / back-end split.

The vehicle code only ever calls into the Library's (aka sensor driver's) front-end.

The front-end has pointers to each back-end.  These pointers are normally held within an array named _drivers[].
On start-up the front-end creates one or more backends based either on automatic detection of the sensor (i.e. probing for a response on a known I2C address) or by using the user defined _TYPE params (i.e. RNGFND_TYPE, RNGFND_TYPE2).
