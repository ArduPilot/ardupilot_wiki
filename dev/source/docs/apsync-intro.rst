.. _apsync-intro:

======
APSync
======

APSync is a project sponsored by `EnRoute <http://enroute.co.jp/>`__ which aims to simplify the setup of companion computer so that they can be used to provide additional functionality to ArduPilot and also to ease integration of UAVs with internet services.

The first beta has started with support the first section listed below (Wifi Access Point & DataFlash logging) for the :ref:`RPi3 <raspberry-pi-via-mavlink>`, :ref:`NVidia TX1 <companion-computer-nvidia-tx1>` and :ref:`Intel Edison <intel-edison>`.

:ref:`Installation instructions <apsync-intro-installing-apsync>` are at the bottom of this page.

.. warning::

   The APSync project is still in beta.  Only the first stage (Wifi Access Point & DataFlash logging) is complete.
   Please get involved by reporting issues in the `ArduPilot Forums <http://discuss.ardupilot.org/>`__ and/or help improve APSync by contacting the developers on `Gitter's ArduPilot/companion chat room <https://gitter.im/ArduPilot/companion>`__!

Wifi Access Point & DataFlash logging
=====================================

.. image:: ../images/apsync-wifiap-dflogger.png
    :target: ../_images/apsync-wifiap-dflogger.png

On start-up an access point is created with name "ardupilot" and password "enRouteArduPilot".

The user can connect to this access point and then easily connect to ardupilot running on the flight controller by setting their ground station (including Mission Planner) to connect using "UDP".

Dataflash logs are streamed to the companion computer via mavlink and stored on the companion computer's filesystem (as well as on the pixhawk's dataflash).
Dataflash log files can then be quickly downloaded (over wifi) using a script (Windows users may use `apsync-download-logs <http://firmware.ardupilot.org/Companion/apsync-download-logs-latest.zip>`__) or you may pull the SD card out of the companion computer.

Data Syncronisation with Web server or Corporate server
=======================================================

.. image:: ../images/apsync-wifiap-datasync.png
    :target: ../_images/apsync-wifiap-datasync.png

The contents of a configurable list of directories will be automatically uploaded to a configurable web server address.

This should allow the pilot to simply bring the vehicle back in range of a trusted wifi access point, reboot the vehicle and have it automatically connect and upload all datafiles including logs, pictures, videos.

.. warning::

   The APSync project is still in beta.  This Data Syncronisation portion is not implemented (yet).

Simple Configuration Web page
=============================

.. image:: ../images/apsync-configurator.png
    :target: ../_images/apsync-configurator.png

A light weight webserver will run on the companion computer (perhaps using CherryPy).  The user will be able to connect to the drone using a known URL and change configuration information including:

- wifi access point name and password
- list of trusted wifi access points
- list of log file directories and web server url (i.e. upload target)

.. warning::

   The APSync project is still in beta.  This Simple Configuration portion is not implemented (yet).

Flexible Video
==============

.. image:: ../images/apsync-flexible-video.png
    :target: ../_images/apsync-flexible-video.png

A video manager will run on the companion computer that informs the ground station of which video streams are available.  The live video is streamed from the vehicle to the ground station using UDP to a port specified by the ground station.
This allows more reliable and flexible streaming as the ground station can discover valid formats and control where the stream is sent.

.. warning::

   The APSync project is still in beta.  This Flexible Video portion is not implemented (yet).

.. _apsync-intro-installing-apsync:

Installing APSync
=================

Images for the supported boards can be found in `firmware.ardupilot.org <http://firmware.ardupilot.org/Companion>`__.

Please follow the instructions for installing these images on the wiki page for each board (:ref:`RPi3 <raspberry-pi-via-mavlink>`, :ref:`NVidia TX1 <companion-computer-nvidia-tx1>` and :ref:`Intel Edison <intel-edison>`).

The flight controller (i.e. Pixhawk or similar) should be configured to communicate with the companion computer by setting the following parameters and then reboot the board:

- :ref:`SERIAL2_BAUD <copter:SERIAL2_BAUD>` 921
- :ref:`SERIAL2_PROTOCOL <copter:SERIAL2_PROTOCOL>` 1
- :ref:`LOG_BACKEND_TYPE <copter:LOG_BACKEND_TYPE>` 3
