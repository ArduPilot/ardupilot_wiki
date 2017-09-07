.. _skyrocket:

=========
SkyRocket
=========

   .. image:: ../images/skyrocket-skyvipergpsdrone.png
       :target: ../_images/skyrocket-skyvipergpsdrone.png

The SkyRocket SkyViper GPS streaming drone uses ArduPilot as its main flight control software.  This page provides details for advanced users and potential developers on this RTF drone.

Where to Buy
------------

The `SkyViper GPS Streaming drone <http://sky-viper.com/product/info/V2450GPS-sd>`__ (`version with FPV headset <http://sky-viper.com/product/info/V2450GPS-fpv>`__ is expected to be on sale through Amazon, ToysRUs and Target in the US, Canada, Chile, France, German, Holland, Lithuania, Mexico, New Zealand, Serbia, UK from October 2017.

SkyRocket sells a variety of different drones, be sure to get the GPS Streaming drone which should have a white shell and usually sells for around $150 US.

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

Software
--------

- the drone ships with a custom version of ArduPilot (Copter) which is close to Copter-3.5 but has additional features that will be coming back into master in the near future (notch filter, etc).  `GitHub repo is here <https://github.com/SkyRocketToys/ardupilot>`__.
- live video can be viewed through a SkyRocket app (Mission Planner video support coming soon).
- APWeb (a tiny open source web server) runs on the drone and allows easy calibration, modification of parameters, uploading firmware to both the drone and transmitter, downloading log files, pictures and videos.
- the transmitter has a relatively low powered STM8 processor running open source software written by Tridge (incorporates some code from Paparazzi).  `Github repo is here <https://github.com/SkyRocketToys/STM8TX>`__.

Videos
------

`sUAS news <https://www.suasnews.com/>`__ interview with Tridge and Matt (from SkyRocket):

..  youtube:: 3RdEELDIeVs
    :width: 100%

Bo Lorentzen's intro video:

..  youtube:: T_ccNYEeviE
    :width: 100%

:ref:`Throw Mode <throw-mode>` demonstrations

..  youtube:: L8xTlBnlu9E
    :width: 100%

..  youtube:: YujYnQHEqQw
    :width: 100%

Calibration using the Web Interface

..  youtube:: VIf9eEwTKkk
    :width: 100%

SkyViper GPS web server demo

..  youtube:: vd0KGiwVd2g
    :width: 100%

Customising transmitter tunes

..  youtube:: ch4o8Rt5pB0
    :width: 100%

Mission Planner Simulator connection to SkyViper

..  youtube:: 1TmlVwTiBRg
    :width: 100%

Adjusting Transmitter Buzzer Frequency

..  youtube:: TI5arJSzYX8
    :width: 100%
