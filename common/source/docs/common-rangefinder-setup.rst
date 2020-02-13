.. _common-rangefinder-setup:

===========================
RangeFinders Setup Overview
===========================

There are many different kinds of rangefinders: Lidar (using laser or infra-red light to measure distance), Sonar (using ultrasonic sound), and Radar (using microwave RF). Some are analog, producing  pulses whose timing represent the distance to an object, others are digital sending data streams over serial to UARTs, or I2C, or even via UAVCAN.

:ref:`RangeFinders (Sonar or Lidar) <common-rangefinder-landingpage>` can be used for :ref:`Object Avoidance <common-object-avoidance-landing-page>` in Copter-3.5 (and higher) in Loiter and AltHold modes and in Rover-3.5 (and higher). As well as altitude sensors for precision landing in Plane and Copter

..  youtube:: y2Kk6nIily0
    :width: 100%

.. warning::
   :ref:`common-object-avoidance-landing-page` features are new and should be used with caution.
   

Up to 10 Rangefinders can be used in the system (1 thru A). :ref:`Object Avoidance <common-object-avoidance-landing-page>` can use a single 360 degree Lidar, or up to 9 unidirectional rangefinders: 8 arranged in a circle covering 45 degree wide segments, plus an upward facing rangefinder. And a downward facing rangefinder used for low altitude height above ground measurements.

Connecting and Configuring the Rangefinder
==========================================

- Follow the instructions for each type of rangefinder described in its linked page on :ref:`common-rangefinder-landingpage`.
- Set the RNGFNDx_ORIENT parameters (i.e. :ref:`RNGFND1_ORIENT <RNGFND1_ORIENT>`, :ref:`RNGFND2_ORIENT <RNGFND2_ORIENT>`, etc.) to specify the direction each range finder is pointing in. 
[site wiki="copter,rover"]
  - For Copter and Rover, up to 8 rangefinders may placed around the vehicle to provide 360 degree coverage, or a single 360 degree Lidar. But only one forward facing rangefinder is required for :ref:`Object Avoidance <common-object-avoidance-landing-page>` .
  - Exclusion zones are provided for 360 degree Lidars, since these may have obstructions in their field of view. Exclusion zones are set via the PRX_IGN_ANGx and PRX_IGN_WIDx parameters, specifying a direction and width the frame obstruction presents and will be ignored. Up to 6 exclusion sectors can be specified.
  - If a rangefinder is oriented facing up, then it will be used in the :ref:`common-simple-object-avoidance` operation as an upwards sensing proximity sensor in Copter LOITER, ALTHOLD, and POSHOLD modes.
[/site]
  - If a rangefinder is oriented facing down, it will automatically be used to for height above ground measurements when within its range for increased landing precision in Copter and Plane autolandings. In Copter, when landing and within 10m of the ground according to the range finder, the vehicle will slow it's descent to the :ref:`LAND_SPEED<LAND_SPEED>` (which defaults to 50cm/s). In Plane, see the "using a rangefinder" section of :ref:`automatic-landing`


.. note:: Only downward facing rangefinders are supported in Plane currently.


References
==========

- Object Avoidance wiki page is :ref:`here <common-object-avoidance-landing-page>`
[site wiki="copter,rover"]
- More details of the algorithms used in :ref:`common-simple-object-avoidance` are on the :ref:`developer wiki's object avoidance page <code-overview-object-avoidance>`
[/site]

