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

   Lidar's are used in flight modes which have height control, such
   as Altitude Hold, Loiter and PosHold Mode.  The data from the sensor
   will be used until you exceed RNGFND_MAX_CM, after that it switches to
   the barometer.  Copter-3.4 includes support for :ref:`Terrain Following <terrain-following>` in Auto mode.
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

    Lightware SF02 <common-rangefinder-sf02>
    LightWare SF10 Lidar <common-lightware-sf10-lidar>
    Maxbotic I2C Sonar <common-rangefinder-maxbotixi2c>
    Maxbotic Analog Sonar <common-rangefinder-maxbotix-analog>
    TeraRanger One Rangefinder <common-teraranger-one-rangefinder>
    LIDAR-Lite <common-rangefinder-lidarlite>
    
[site wiki="copter"]
    Analog Sonar (AC3.1) <sonar>
[/site]

