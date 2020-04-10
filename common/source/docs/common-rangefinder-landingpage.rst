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

Follow the links below (or in sidebar) for configuration information
based upon your set-up.

.. image:: ../../../images/RangeFinder_LandingPageImage_4.jpg
    :target: ../_images/RangeFinder_LandingPageImage_4.jpg


.. toctree::
    :maxdepth: 1
    
[site wiki="rover"]
    Sonar Sensors <sonar-sensors>
[/site]

    LightWare SF20 / LW20 Lidar <common-lightware-lw20-lidar>
    LightWare SF10 / SF11 Lidar <common-lightware-sf10-lidar>
    Lightware SF02 Lidar <common-rangefinder-sf02>
    Lightware SF40c (360 degree) <common-lightware-sf40c-objectavoidance>
    Maxbotic I2C Sonar <common-rangefinder-maxbotixi2c>
    Maxbotic Analog Sonar <common-rangefinder-maxbotix-analog>
    TeraRanger One Rangefinder <common-teraranger-one-rangefinder>
    TerraRanger Tower (360 degree) <common-teraranger-tower-objectavoidance>
    Leddar One Lidar <common-leddar-one-lidar>
    LIDAR-Lite <common-rangefinder-lidarlite>
    ST VL53L0X I2C LiDAR <http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html>
    
[site wiki="copter"]
    Analog Sonar (AC3.1) <sonar>
[/site]

