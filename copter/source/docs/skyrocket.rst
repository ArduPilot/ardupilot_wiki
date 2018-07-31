.. _skyrocket:

=========
SkyRocket
=========

   .. image:: ../images/skyrocket-skyvipergpsdrone.png
       :target: ../_images/skyrocket-skyvipergpsdrone.png

A number of SkyRocket drones use ArduPilot as flight control software.  This page provides details for advanced users and potential developers on these RTF drones.

Where to Buy
------------

2017 models.  The `SkyViper GPS Streaming drone <http://sky-viper.com/product/info/V2450GPS-sd>`__ (`version with FPV headset <http://sky-viper.com/product/info/V2450GPS-fpv>`__) is on sale through `Target <https://www.target.com/p/sky-viper-streaming-video-drone-v2450hd/-/A-52136996>`__, `Walmart <https://www.walmart.com/ip/Sky-Viper-Streaming-Drone-with-GPS/797973157>`__, `Amazon <https://www.amazon.com/Sky-Viper-v2450GPS-Streaming-Autopilot/dp/B072HH13VQ/ref=lp_13203361011_1_6>`__ and Costco in the US, Australia, Canada, Chile, France, German, Holland, Lithuania, Mexico, New Zealand, Serbia, UK since October 2017.

SkyRocket sells a variety of different drones.  Of the 2017 models, only the V2450GPS Streaming drone (which will have a white shell) is capable of running ArduPilot.

2018 models.  In August 2018, SkyRocket released three new ArduPilot powered drones, the Fury, Scout, and Journey. The Fury and Scout utilize "SurfaceScan", built on ArduPilot's OpticalFlow feature, to allow for indoor flight stability and position holding, unprecendented in this price range. The Journey, with GPS rather than OpticalFlow, takes lessons learned from the V2450GPS and builds on those for a solid re-release for the full GPS-enabled ArduPilot experience. The flight parameters and firmware for both the Scout and Journey can be easily modified as the user sees fit through the embedded wifi access point, powered by APWeb or using your favorite ground control software.  Much of the information on this page, which was written for the V2450GPS, will at least partially apply to these new models, with updates annotated as they are discovered.

Discussion
----------

We have a `SkyViper section <https://discuss.ardupilot.org/c/arducopter/skyviper>`__ on the ArduPilot discussion forum. Have a look there to see what people are up to and ask questions.

More developer specific information including how to compile the sonix and ardupilot firmware can be found on the `developer wiki <http://ardupilot.org/dev/docs/skyviper.html>`__.

Toy Mode
--------

The Skyviper has Toy Mode set by default. Toy Mode handles the following functions, specific to the Sky Viper 2450GPS:

- Handles the button presses from the transmitter
- Magically trims the sticks when they're idle and the SV is disarmed
- Toggles the fence on and off depending on the situation
- Basically, if GPS is "good", the fence is armed, if GPS is "not good", the fence is disarmed (for obvious reasons)
- Toy Mode also handles moving you between ALT_HOLD and LOITER automatically depending on GPS Status.
- Handles the LED bling (source: peterbarker)
- Toy Mode automatically adjusts the thrust based on voltage
- Toy Mode processes an arming script when armed using the throttle control to prevent sudden climbs (idles motors for a moment before increasing speed)
- Performs some automatic Compass tuning if needed

Hardware
--------

- STM32 CPU
- 5x serial ports
- 1x I2C
- 1x SPI
- ICM20789 IMU including 3-axis accelerometer, gyro and barometer
- Ublox M8 GPS
- 1S battery (4.2V max, replacement batteries are readily available on Amazon and other places)
- brushed motors; 8.5x20mm with a kV between 16,000 and 17,000 as measured. 
- The small pinion is 13T and the larger one is 73T, which provides a gear ratio of approximately 5.6
- the camera can be manually adjusted to point forward, down or anywhere in between
- 2.4Ghz wifi for telemetry and video
- 145g
- flight time of about 11min
- top speed of between 8m/s ~ 10m/s
- video streaming uses a Sonix board with ARM CPU running FreeRTOS and OmniVision OV9732 chip

.. image:: ../../../images/skyrocket-flight-controller.png
    :target: ../_images/skyrocket-flight-controller.png




`sUAS news <https://www.suasnews.com/>`__ interview with Tridge and Matt (from SkyRocket):

..  youtube:: 3RdEELDIeVs
    :width: 100%

More Info
---------

.. toctree::
    :maxdepth: 1

    Videos <skyrocket-videos>
    Hardware <skyrocket-hardware>
    Throw Mode <skyrocket-throw>
    Using other Ground Stations <skyrocket-gcs>
    Uploading Software <skyrocket-software>
