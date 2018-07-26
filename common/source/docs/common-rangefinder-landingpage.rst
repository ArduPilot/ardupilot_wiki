.. _common-rangefinder-landingpage:

===========================
Rangefinders (landing page)
===========================

Copter/Plane/Rover support a number of different rangefinders including
Maxbotix Sonar and Pulsed Light LED range finders.

[site wiki="plane"]
.. tip::

   If you get a "Bad LiDAR Health" message in *Mission Planner* this
   is caused because the rangefinder isn't enabled for landing.  This is
   done through the ``RNGFND_LANDING`` parameter and if set you should see
   the message disappear.
[/site]

[site wiki="copter"]
.. note::

   Downward facing Lidar are used in flight modes which have height control, such
   as Altitude Hold, Loiter and PosHold Mode.  The data from the sensor
   will be used until you exceed RNGFND_MAX_CM, after that it switches to
   the barometer.

   Copter-3.4 (and higher) includes support for :ref:`Terrain Following <terrain-following>` in Auto mode.

   Copter-3.5 (and higher) can use lidar for :ref:`object avoidance <common-rangefinder-objectavoidance>`.
[/site]

   
.. warning::
   
   RNGFND_MAX_CM must be set to a tested, appropriate value.  If RNGFND_MAX_CM is set to a value
   greater than the range of the sensor, the flight controller will not respond correctly to the 
   data provided.

Follow the links below (or in sidebar) for configuration information
based upon your set-up.

.. image:: ../../../images/RangeFinder_LandingPageImage_4.jpg
    :target: ../_images/RangeFinder_LandingPageImage_4.jpg


.. toctree::
    :maxdepth: 1
    
[site wiki="rover"]
    Sonar Sensors <sonar-sensors>
[/site]

    Attollo Engineering Wasp200 <common-wasp200-lidar>
    Benewake TF02 <common-benewake-tf02-lidar>
    Benewake TFmini <common-benewake-tfmini-lidar>
    EchoLogger ECT400 <common-echologger-ect400>
    LightWare SF20 / LW20 Lidar <common-lightware-lw20-lidar>
    LightWare SF10 / SF11 Lidar <common-lightware-sf10-lidar>
    Lightware SF02 Lidar <common-rangefinder-sf02>
[site wiki="copter"]
    Lightware SF40c (360 degree) <common-lightware-sf40c-objectavoidance>
[/site]
    Maxbotic I2C Sonar <common-rangefinder-maxbotixi2c>
    Maxbotic Analog Sonar <common-rangefinder-maxbotix-analog>
    TeraRanger One Rangefinder <common-teraranger-one-rangefinder>
[site wiki="copter"]
    TerraRanger Tower (360 degree) <common-teraranger-tower-objectavoidance>
[/site]
    Leddar One Lidar <common-leddar-one-lidar>
    LIDAR-Lite <common-rangefinder-lidarlite>
    RPLidar A2 360 degree laser scanner <common-rplidar-a2>
    ST VL53L0X Lidar <common-vl53l0x-lidar>

[site wiki="copter"]
    Analog Sonar (AC3.1) <sonar>
[/site]

