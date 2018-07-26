.. _common-rangefinder-objectavoidance:

===============================================
RangeFinders (Sonar/Lidar) for Object Avoidance
===============================================

[copywiki destination="copter"]

:ref:`RangeFinders (Sonar or Lidar) <common-rangefinder-landingpage>` can be used for Object Avoidance in Copter-3.5 (and higher) in Loiter and AltHold modes.

..  youtube:: y2Kk6nIily0
    :width: 100%

.. warning::

   This feature is new for Copter-3.5 and should be used with caution.
   Only up to two rangefinders can be used at the same time although a `minor code change <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/RangeFinder.h#L24>`__ can allow up to 4 to be used.
    
Connecting and Configuring the Lidar
====================================

- follow the normal :ref:`instructions for setting up rangefinders <common-rangefinder-landingpage>`.
- set the RNGFNDx_ORIENT parameters (i.e. :ref:`RNGFND_ORIENT <RNGFND_ORIENT>`, :ref:`RNGFND2_ORIENT <RNGFND2_ORIENT>`) to specify the direction the range finder is pointing in (i.e. 0=Forward, 2=Right, 4=Back, 6=Left, 24=Up, 25=Down)
- set :ref:`PRX_TYPE <PRX_TYPE>` = "4" to enable using range finders as "proximity sensors".
- set :ref:`AVOID_ENABLE <AVOID_ENABLE>` to "7" to enable avoidance using proximity sensors (and fences)

References
==========

- :ref:`Lightware SF40c <common-lightware-sf40c-objectavoidance>` (the first lidar used to add object avoidance) has details about limitations of the current implementation.
- More details of the algorithms used are on the `developer wiki's object avoidance page <http://ardupilot.org/dev/docs/code-overview-object-avoidance.html>`__.
