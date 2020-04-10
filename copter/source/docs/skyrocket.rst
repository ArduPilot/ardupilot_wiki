.. _skyrocket:

=========
SkyRocket
=========

   .. image:: ../images/skyrocket-skyvipergpsdrone.png
       :target: ../_images/skyrocket-skyvipergpsdrone.png

The SkyRocket SkyViper GPS streaming drone uses ArduPilot as its main flight control software.  This page provides details for advanced users and potential developers on this RTF drone.

Where to Buy
------------

The `SkyViper GPS Streaming drone <http://sky-viper.com/product/info/V2450GPS-sd>`__ (`version with FPV headset <http://sky-viper.com/product/info/V2450GPS-fpv>`__) is on sale through `Amazon <https://www.amazon.com/Sky-Viper-v2450GPS-Streaming-Autopilot/dp/B072HH13VQ/ref=lp_13203361011_1_6>`__, `ToysRUs <https://www.toysrus.com/product?productId=125702746>`__ (`with FPV headset <https://www.toysrus.com/buy/video-drones/sky-viperr-v2450-gps-streaming-video-drone-with-fpv-headset-816322018322-134534546>`__), `Target <https://www.target.com/p/sky-viper-streaming-video-drone-v2450hd/-/A-52136996>`__ and Costco in the US, Australia, Canada, Chile, France, German, Holland, Lithuania, Mexico, New Zealand, Serbia, UK since October 2017.

SkyRocket sells a variety of different drones, be sure to get the GPS Streaming drone which should have a white shell and usually sells for around $150 US.

Discussion
----------

We have a `SkyViper section <https://discuss.ardupilot.org/c/arducopter/skyviper>`__ on the ArduPilot discussion forum. Have a look there to see what people are up to and ask questions.

More developer specific information including how to compile the sonix and ardupilot firmware can be found on the `developer wiki <http://ardupilot.org/dev/docs/skyviper.html>`__.

Hardware
--------

- STM32 CPU
- 5x serial ports
- 1x I2C
- 1x SPI
- ICM20789 IMU including 3-axis accelerometer, gyro and barometer
- Ublox M8 GPS
- 1S battery (4.2V max, replacement batteries are readily available on Amazon and other places)
- brushed motors
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
    Throw Mode <skyrocket-throw>
    Using other Ground Stations <skyrocket-gcs>
    Uploading Software <skyrocket-software>
