.. _common-rangefinder-setup:

===========================
RangeFinders Setup Overview
===========================

There are many different kinds of rangefinders: Lidar (using laser or infra-red light to measure distance), Sonar (using ultrasonic sound), and Radar (using microwave RF). Some are analog, producing  pulses whose timing represent the distance to an object, others are digital sending data streams over serial to UARTs, or I2C, or even via DroneCAN.

:ref:`RangeFinders (Sonar or Lidar) <common-rangefinder-landingpage>` can be used for :ref:`Object Avoidance <common-object-avoidance-landing-page>` as well as altitude sensors for precision landing in Plane and Copter.

..  youtube:: y2Kk6nIily0
    :width: 100%

.. warning::
   :ref:`common-object-avoidance-landing-page` features are new and should be used with caution.
   

Up to 10 Rangefinders can be used in the system (1 thru A). :ref:`Object Avoidance <common-object-avoidance-landing-page>` can use a single 360 degree Lidar, or up to 9 unidirectional rangefinders: 8 arranged in a circle covering 45 degree wide segments, plus an upward facing rangefinder. And a downward facing rangefinder used for low altitude height above ground measurements.

Connecting and Configuring the Rangefinder
==========================================

- Follow the instructions for each type of rangefinder described in its linked page on :ref:`common-rangefinder-landingpage`.
- Set the RNGFNDx_ORIENT parameters (i.e. :ref:`RNGFND1_ORIENT <RNGFND1_ORIENT>`, :ref:`RNGFND2_ORIENT <RNGFND2_ORIENT>`, etc.) to specify the direction each range finder is pointing in. 

.. note:: Note that if the type of rangefinder is set or changed, a reboot will be required.


[site wiki="copter,rover"]
  - Set the parameter ``PRXx_TYPE`` = 4 for each horizontal rangefinder(s) to use them as Proximity Sensors (For obstacle avoidance).
  - For Copter and Rover, up to 8 rangefinders may placed around the vehicle to provide 360 degree coverage, or a single 360 degree Lidar. But only one forward facing rangefinder is required for :ref:`Object Avoidance <common-object-avoidance-landing-page>`.
  - If a rangefinder is oriented facing up, then it will be used in the :ref:`common-simple-object-avoidance` operation as an upwards sensing proximity sensor in Copter LOITER, ALTHOLD, and POSHOLD modes.
[/site]
[site wiki="plane,copter"]
  - If a rangefinder is oriented facing down, it will  be used to for height above ground measurements when within its range for increased landing precision in Copter, and in Plane autotakeoffs/autolandings, and QuadPlane VTOL operations depending on which bits in the :ref:`RNGFND_LANDING<RNGFND_LANDING>` parameter are set. In Copter, when landing and within 10m of the ground according to the range finder, the vehicle will slow its descent to the :ref:`LAND_SPD_MS<LAND_SPD_MS>` (which defaults to 50cm/s). In Plane, see the "using a rangefinder" section of :ref:`automatic-landing`

  - If using a rangefinder for altitude measurements, be sure to read this page:

.. toctree::
    :maxdepth: 1

    Understanding Altitude in ArduPilot <common-understanding-altitude>


.. note:: Only downward facing rangefinders are supported in Plane currently.
[/site]

MAVLink Rangefinders
====================

Rangefinders of type MAVLink (``RNGFNDx_TYPE`` = 10) are not read directly by the autopilot: their distances arrive in `DISTANCE_SENSOR <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`__ messages, normally sent by a companion computer or by a smart sensor over a MAVLink serial port.

The ``RNGFNDx_ADDR`` parameters (i.e. :ref:`RNGFND1_ADDR <RNGFND1_ADDR>`, :ref:`RNGFND2_ADDR <RNGFND2_ADDR>`, etc.) select which sensor each rangefinder accepts messages from:

- ``RNGFNDx_ADDR`` = 0 (the default): any ``DISTANCE_SENSOR`` message whose orientation matches ``RNGFNDx_ORIENT`` is accepted, whatever its ``id``.
- ``RNGFNDx_ADDR`` non-zero: only messages whose ``id`` field also matches that value are accepted.

To use more than one MAVLink rangefinder pointing in the same direction, give each a distinct non-zero ``RNGFNDx_ADDR`` and have the sender put the matching value in the ``id`` field of that sensor's messages. Otherwise every MAVLink rangefinder with that orientation accepts every message and they cannot be told apart. ``RNGFNDx_ADDR`` accepts 1 to 127, so the sender must use an ``id`` in that range.

Each rangefinder then keeps and logs its own reading, but sensors facing the same direction are not combined: features which ask for the distance in a given direction use the first rangefinder with that orientation which is reading normally, so a second one facing the same way acts as a backup rather than adding to the first.

.. warning:: ``RNGFNDx_ADDR`` was previously ignored by MAVLink rangefinders. If a rangefinder is switched to ``RNGFNDx_TYPE`` = 10 from a type which uses ``RNGFNDx_ADDR``, such as I2C or DroneCAN, reset ``RNGFNDx_ADDR`` to 0 unless the sender really does use that value as its ``id``, otherwise no distances will be accepted.

.. note:: This controls only which incoming messages the MAVLink rangefinder accepts. It does not affect the ``DISTANCE_SENSOR`` messages the autopilot sends out, in which the ``id`` field is the rangefinder's instance number (0 for ``RNGFND1_``, 1 for ``RNGFND2_``, and so on), nor the separate ``DISTANCE_SENSOR`` messages generated from proximity sensors, which use ids of 10 and above. It also has no effect on MAVLink proximity sensors (``PRXx_TYPE`` = 2), which sort incoming messages by orientation alone.

References
==========

- Object Avoidance wiki page is :ref:`here <common-object-avoidance-landing-page>`
[site wiki="copter,rover"]
- More details of the algorithms used in :ref:`common-simple-object-avoidance` are on the :ref:`developer wiki's object avoidance page <code-overview-object-avoidance>`
[/site]

