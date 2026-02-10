.. _common-rangefinder-landingpage:

===========================
Rangefinders (landing page)
===========================

Copter/Plane/Rover support a number of different rangefinders including Lidars (which use lasers or infra-red beams for distance measurements), 360 degree Lidars (which can detect obstacles in multiple directions) and Sonars (which use ultrasonic sound). This category also includes Maxbotix Sonar and Pulsed Light LED range finders. These devices can be used for measuring distance near to  the ground for precision landings and altitude control, water depth, or object distance as proximity sensors for :ref:`Object Avoidance<common-object-avoidance-landing-page>`. Vision systems (see :ref:`common-realsense-depth-camera`) can also be used for Object Avoidance.

A forward facing rangefinders can also be used for Obstacle Avoidance. See Rangefinder :ref:`Setup Overview <common-rangefinder-setup>` to know more.

.. warning:: Many rangefinders are not included in the standard builds. If you get a pre-arm message that it is not being detected on power up, check the firmware's included :ref:`features list <common-limited_firmware>`

[site wiki="plane"]
.. tip::

   If you get a "Bad LiDAR Health" message in *Mission Planner* this
   is caused because the rangefinder isn't enabled for landing.  This is
   done through the ``RNGFND_LANDING`` parameter and if set you should see
   the message disappear.
[/site]

[site wiki="copter"]
.. note::

   Downward facing rangefinders are automatically used in flight modes which have height control, such
   as Altitude Hold, Loiter and PosHold Mode.  The data from the sensor
   will be used until you exceed ``RNGFNDx_MAX``, after that it switches to
   the barometer.

   Copter includes support for :ref:`Terrain Following <terrain-following>` in Auto mode.

   Copter can also use rangefinders for :ref:`Object Avoidance <common-object-avoidance-landing-page>`.
[/site]

   
.. warning::
   
   ``RNGFNDx_MAX`` must be set to a tested, appropriate value.  If ``RNGFNDx_MAX`` is set to a value
   greater than the range of the sensor, the autopilot will not respond correctly to the 
   data provided.

Follow the links below (or in sidebar) for configuration information
based upon your set-up.

.. image:: ../../../images/RangeFinder_LandingPageImage_4.jpg
    :target: ../_images/RangeFinder_LandingPageImage_4.jpg


.. toctree::
    :maxdepth: 1

    Rangefinder Setup Overview <common-rangefinder-setup>
[site wiki="copter,rover"]

    Proximity Sensors <common-proximity-landingpage>
    
[/site]

Unidirectional Rangefinders
===========================

.. toctree::
    :maxdepth: 1

    Ainstein US-D1 Radar Altimeter <common-aerotenna-usd1>
    Ainstein LR-D1 Radar Altimerer <common-ainstein-lrd1>
    ARK DIST MR <common-ark-dist-mr>    
    ARK DIST SR <common-ark-dist-sr>
    Attollo Engineering Wasp200 <common-wasp200-lidar>
    Avionics Anonymous DroneCAN LIDAR Interface <common-avanon-laserint>
    Benewake TF02-Pro / TF03 / TFS20-L / TF-Luna / TF-Nova / TF350 <common-benewake-tf02-lidar>
    Benewake TFmini / TFmini Plus <common-benewake-tfmini-lidar>
    Garmin Lidar-Lite <common-rangefinder-lidarlite>
    GY-US42 Sonar <common-rangefinder-gy-us42>
    Hexsoon 24G Radar <common-rangefinder-hexsoon-24g>
    HC-SR04 Sonar <common-rangefinder-hcsr04>
    JAE JRE-30 <common-rangefinder-jae-jre-30>
    LeddarTech Leddar One <common-leddar-one-lidar>
    LeddarTech LeddarVu8 <common-leddartech-leddarvu8-lidar>
    LightWare SF10 / SF11 Lidar <common-lightware-sf10-lidar>
    LightWare SF20 / LW20 Lidar <common-lightware-lw20-lidar>
    Lightware SF02 Lidar <common-rangefinder-sf02>
    Maxbotix I2C Sonar <common-rangefinder-maxbotixi2c>
    Maxbotix Analog Sonar <common-rangefinder-maxbotix-analog>
    Nanoradar NRA24 <common-rangefinder-nra24>
    Nooploop TOF-Sense P <common-rangefinder-nooploop-tofsense-p>
    Nooploop TOF-Sense F <common-rangefinder-nooploop-tofsense-f>
    ST VL53L0X / VL53L1X Lidar <common-vl53l0x-lidar>
    TeraRanger One/EVO Rangefinders <common-teraranger-one-rangefinder>
    TeraRanger NEO <common-teraranger-neo>

Underwater Sonar
================
.. toctree::
    :maxdepth: 1

    Underwater Sonars <common-underwater-sonars-landingpage>

Omnidirectional Proximity Rangefinders
======================================

.. toctree::
    :maxdepth: 1

    Hexsoon 77G MWW Radar <common-rangefinder-hexsoon-radar>
    LDRobot LD-06 TOF <common-ld06>
    Lightware SF40/C (360 degree) <common-lightware-sf40c-objectavoidance>
    Lightware SF45/B (350 degree) <common-lightware-sf45b>
    Nanoradar MR72 (112 degree) <common-rangefinder-mr72>
    RPLidar A2/C1/S1 360 degree Laser/TOF LIDAR <common-rplidar-a2>
    TerraRanger Tower/ Tower EVO (360 degree) <common-teraranger-tower-objectavoidance>
    Cygbot D1 (120 degree) <https://www.cygbot.com/_files/ugd/f5911d_726a54fc4f6644bcbec0d9b00236ffda.pdf>

[site wiki="copter,rover"]
Vision Based Sensors
====================

.. toctree::
    :maxdepth: 1
    
    Intel Realsense Depth Camera <common-realsense-depth-camera>
[/site]



