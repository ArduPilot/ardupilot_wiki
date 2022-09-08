.. _common-simple-object-avoidance:

[copywiki destination="copter,rover"]
=======================
Simple Object Avoidance
=======================

Copter supports simple object avoidance horizontally and upward, while Rover avoids simple objects by only stopping. Both use proximity sensors such as:

- 360 degree lidar including the :ref:`Lightware SF40C <copter:common-lightware-sf40c-objectavoidance>`, :ref:`TeraRanger Tower <copter:common-teraranger-tower-objectavoidance>` or :ref:`RPLidarA2/A3 <copter:common-rplidar-a2>`. See the :ref:`Proximity Sensor page<common-proximity-landingpage>` for more details.
- Any of the supported :ref:`Rangefinders <common-rangefinder-landingpage>`.As of ArduPilot firmware versions 4.0 and higher, up to 9 rangefinders can be used for object avoidance. See :ref:`common-rangefinder-setup` for more information.
- Sensors capable of providing `MAVLink Distance Sensor <https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR>`__ messages (like `OpenKai with a 3D camera <https://www.youtube.com/watch?v=qk_hEtRASqg>`__)
- 3D Obstacle Avoidance via the new Mavlink message `OBSTACLE_DISTANCE_3D <https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D>`__ . Depth cameras can use this message.


..  youtube:: BDBSpR1Dw_8
    :width: 100%


In addition, Simple Object Avoidance can use Geo-Fences and Fence Beacons as proximity sensors as determined by the setting of the :ref:`AVOID_ENABLE<AVOID_ENABLE>` parameter.


Simple Object Avoidance Algorithms
==================================

Horizontal object avoidance works in :ref:`AltHold <altholdmode>` and :ref:`Loiter <loiter-mode>` modes.  Upward object avoidance works in LOITER, ALTHOLD modes only.

.. warning:: Only one proximity sensor source can be enabled, using either a 360 degree lidar or up to 9 rangefinders.

.. note:: For Object Avoidance in AUTO and GUIDED modes in Copter and Rover, see :ref:`common-oa-bendyruler` or :ref:`common-oa-dijkstras`

Details on how simple object avoidance is implemented for Copter in ALTHOLD and LOITER mode can be found :ref:`here in the developer wiki <dev:code-overview-object-avoidance>` and involves assessing all the objects detected, in all reported quadrants, and adds control input to the pilot's,trying to move away from the aggregate threat or stop.

In ALTHOLD mode, the aggregate threat is translated into an attempt to add lean input into the pilot's commands to move the Copter away from the aggregate obstructions. The pilot can still overcome these additions to his command inputs and fly into a object, if determined to do so.

In LOITER, either stopping in front of the object or a "sliding" algorithm is used to avoid it. "Sliding" involves slowing and slightly yawing as the vehicle approaches the object. For example, if Copter approaches a fence line at an angle, it will "slide along" the fence as the operator pushes the Copter toward the fence. Head-on approaches would stop, even if the pilot continues to "pushes" it forward.

Setup the Proximity Sensor
==========================

For lidars follow the instructions corresponding to the lidar on the vehicle on the :ref:`Proximity Sensor documentation<common-proximity-landingpage>`.

  - :ref:`Lightware SF40C <common-lightware-sf40c-objectavoidance>`
  - :ref:`TeraRanger Tower/ Tower EVO <common-teraranger-tower-objectavoidance>`
  - :ref:`RPLidarA2/A3 <common-rplidar-a2>`

For other rangefinders follow the instructions found in there individual setup pages :ref:`here <common-rangefinder-landingpage>`

Be sure to read the :ref:`common-rangefinder-setup` page


SAFETY FIRST!
=============

- The avoidance algorithms have been constantly changing. While in most scenarios it will help the user keep the vehicle safe from any obstacles, or fence breaches; due to unknown sensor glitch, or other such problems, avoidance should be swiftly turned off mid-flight (especially while trying for the first time).
- Set any vacant channel of your Transmitter to use RCx_OPTION parameter and set it to 40. For example, if channel 8 switch of your transmitter is vacant, set :ref:`RC8_OPTION<RC8_OPTION>` = 40.
- Toggling this switch to HIGH would switch on Proximity based avoidance and vice versa.


Configuring Simple Avoidance for Copter in Loiter Mode
======================================================
Example setup below shown for first proximity sensor:

- set :ref:`AVOID_ENABLE <AVOID_ENABLE>` = 7 ("All") to use all sources of barrier information including "Proximity" sensors
- set :ref:`PRX1_TYPE <PRX1_TYPE>` to a 360 deg Lidar type being used or = 4, to enable using a range finders as a "proximity sensor"
- in :ref:`Loiter <loiter-mode>`

  - :ref:`AVOID_MARGIN <AVOID_MARGIN>` controls how many meters from the barrier the vehicle will attempt to stop or try to slide along it
  - :ref:`AVOID_BEHAVE <AVOID_BEHAVE>` allows setting whether the vehicle should simply Stop in front of the barrier or Slide around it. This parameter only affects Copter, since Rover always stops.

Configuring Simple Avoidance for Rover
======================================
Example setup below shown for first proximity sensor:

- set :ref:`AVOID_ENABLE <AVOID_ENABLE>` = 7 ("All") to use all sources of barrier information including "Proximity" sensors.
- set :ref:`PRX1_TYPE <PRX1_TYPE>` = "4" to enable using first range finder as a "proximity sensor"
- Rover attempts to stop the vehicle before it hits objects in all modes except MANUAL.

"Stop" Avoidance
----------------


..  youtube:: ho9mlVwhgHA
    :width: 100%




Advanced Configuration for Simple Avoidance (Copter/Rover 4.1 and above)
========================================================================

Backing away from obstacles
---------------------------

The vehicle will actively attempt to maintain distance (margin) from obstacles. This means that if an obstacle approaches the vehicle from any direction, and the sensor detects it, the vehicle will back away from the obstacle.


..  youtube:: -6PWB52J3ho
    :width: 100%



..  youtube:: oPI0SUQVDRQ
    :width: 100%



- The speed of this backing away can be controlled via the parameter: :ref:`AVOID_BACKUP_SPD <AVOID_BACKUP_SPD>`
- Setting this parameter to zero would disable backing up. Therefore, the vehicle will attempt to stop or slide in front of the obstacle, but never try and maintain a margin from the obstacle if it was to come closer due to any reason.



Getting smoother avoidance experience
-------------------------------------

- Depending on the tuning of the vehicle, current velocity, distance to the obstacle; the user might feel that while avoidance is active, the vehicle response is very "jerky" and not smooth.
- In this case, user should reduce the maximum acceleration with which the vehicle would avoid obstacles. See the parameter :ref:`AVOID_ACCEL_MAX <AVOID_ACCEL_MAX>`.

.. warning:: Setting :ref:`AVOID_ACCEL_MAX <AVOID_ACCEL_MAX>` too low would mean that the response to the obstacle will be very sluggish and the vehicle may not be able to stop in time. Therefore, be careful while pushing this parameter to extremes.


Minimum altitude (Copter only)
------------------------------

- If the sensor has a wide field of view, on takeoff and landings it might see the ground below as obstacle and the vehicle might react to it. 
- Set the parameter :ref:`AVOID_ALT_MIN <AVOID_ALT_MIN>` to have a minimum altitude before avoidance is switched on.

.. note:: This feature requires a valid Downward Facing Rangefinder reading to works



Configuring Simple Avoidance for Copter in Altitude Hold Mode
=============================================================

.. warning:: While this mode does have the option of avoidance, avoidance in Loiter mode is much more advanced and has more features.

- in :ref:`AltHold <altholdmode>`

  - :ref:`AVOID_DIST_MAX <AVOID_DIST_MAX>` controls how far from a barrier the vehicle starts leaning away from the barrier
  - :ref:`AVOID_ANGLE_MAX <AVOID_ANGLE_MAX>` controls how far the vehicle will try to lean away from the barrier
